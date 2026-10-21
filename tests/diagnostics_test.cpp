#include "tasks/base/ascii_serial.hpp"
#include "tasks/base/diagnostics.hpp"
#include "utils/common/image.hpp"

#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace awakening;

static void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

static std::vector<uint8_t> packet(const std::string& payload) {
    const auto crc=ascii_serial::crc16_mcrf4xx(
        reinterpret_cast<const uint8_t*>(payload.data()),payload.size());
    char suffix[16];
    std::snprintf(suffix,sizeof(suffix),"C%04x%%\n",unsigned(crc));
    const auto frame=payload+suffix;
    return {frame.begin(),frame.end()};
}

static void raw_quaternion_and_protocol() {
    auto frame=packet("ARQ2.0000X0.0000Y0.0000Z0.0000");
    ascii_serial::RecvDiagnostics raw;
    auto q=ascii_serial::parse_recv_frame(frame,&raw);
    check(q.has_value(),"valid CRC quaternion rejected");
    check(raw.crc_ok && raw.has_raw_quaternion,"raw diagnostics missing");
    check(raw.w==2 && raw.norm==2,"raw norm must be measured before normalization");
    check(q->w()==1 && std::abs(q->norm()-1)<1e-12,"control quaternion must remain normalized");
    check(ascii_serial::parse_recv_frame(frame).has_value(),"legacy parser call broke");
    frame[4]='3';
    check(!ascii_serial::parse_recv_frame(frame,&raw),"bad CRC accepted");
    check(!raw.crc_ok && !raw.has_raw_quaternion,"stale raw data on CRC failure");
    check(!ascii_serial::parse_recv_frame(packet("ARQ0.0000X0.0000Y0.0000Z0.0000"),&raw),"zero quaternion accepted");
    check(raw.has_raw_quaternion && raw.norm==0,"zero raw norm lost");
    check(!ascii_serial::parse_recv_frame(packet("ARQnanX0.0000Y0.0000Z0.0000"),&raw),"NaN accepted");
    check(!ascii_serial::parse_recv_frame(packet("ARQinfX0.0000Y0.0000Z0.0000"),&raw),"infinity accepted");
    q=ascii_serial::parse_recv_frame(packet("ARQ0.5X-0.5Y0.5Z-0.5"),&raw);
    check(q && q->w()==.5 && q->x()==-.5 && q->y()==.5 && q->z()==-.5,
          "quaternion component order changed");
}

static void split_and_coalesced_reads() {
    ascii_serial::RecvFrameScanner scanner;
    auto first=packet("ARQ1.0000X0.0000Y0.0000Z0.0000");
    auto second=packet("ARQ0.0000X1.0000Y0.0000Z0.0000");
    size_t parsed=0;
    auto accept=[&](std::vector<uint8_t>&& bytes) {
        check(bool(ascii_serial::parse_recv_frame(bytes)),"scanner emitted invalid frame");
        ++parsed;
    };
    scanner.feed(first.data(),5,accept);
    check(parsed==0,"partial frame emitted too soon");
    std::vector<uint8_t> combined(first.begin()+5,first.end());
    combined.insert(combined.end(),{0x5a,0x00,0x01});
    combined.insert(combined.end(),second.begin(),second.end());
    scanner.feed(combined.data(),combined.size(),accept);
    check(parsed==2,"split or coalesced frame count incorrect");
}

static void camera_metadata_clone() {
    ImageFrame frame;
    frame.src_img=cv::Mat::zeros(2,2,CV_8UC1);
    frame.format=PixelFormat::GRAY;
    frame.timestamp=Clock::now();
    frame.capture_timing.sdk_metadata_available=true;
    frame.capture_timing.host_buffer_ns=123;
    frame.capture_timing.hardware_frame_id=42;
    frame.capture_timing.device_timestamp_ticks=(uint64_t(1)<<40)+15;
    auto copy=frame.clone();
    check(copy.timestamp==frame.timestamp,"clone changed image time");
    check(copy.capture_timing.host_buffer_ns==123 && copy.capture_timing.hardware_frame_id==42
          && copy.capture_timing.device_timestamp_ticks==((uint64_t(1)<<40)+15),
          "clone dropped capture metadata");
    check(copy.src_img.data!=frame.src_img.data,"clone no longer owns separate pixels");
}

static void asynchronous_recorder(const std::filesystem::path& root) {
    record::DiagnosticsRecorder recorder;
    check(!recorder.open(root,false),"disabled recorder opened");
    check(!recorder.write({{"kind","disabled"}}),"disabled recorder accepted event");
    check(recorder.open(root,true,2),"recorder did not open");
    const auto first_path=recorder.path();
    std::atomic<uint64_t> accepted{0};
    std::vector<std::thread> producers;
    for(int p=0;p<4;++p) producers.emplace_back([&,p] {
        for(int i=0;i<1000;++i)
            if(recorder.write({{"kind","test"},{"producer",p},{"index",i}})) ++accepted;
    });
    for(auto& producer:producers)producer.join();
    recorder.close();
    check(recorder.written()==accepted,"accepted diagnostic events were lost on close");
    check(recorder.written()+recorder.dropped()==4000,"overflow accounting incorrect");
    std::ifstream in(first_path);
    std::string line;
    uint64_t actual=0;
    bool status=false;
    uint64_t final_dropped=0;
    while(std::getline(in,line)) {
        auto value=nlohmann::json::parse(line);
        if(value.at("kind")=="test") ++actual;
        if(value.at("kind")=="recorder_status") {
            status=true;
            final_dropped=value.at("dropped").get<uint64_t>();
        }
    }
    check(actual==accepted && status,"JSONL drain/footer missing");
    check(final_dropped==recorder.dropped(),"footer dropped count incorrect");
    check(!recorder.write({{"kind","after_close"}}),"closed recorder accepted event");
    check(recorder.open(root,true),"recorder could not reopen");
    check(first_path!=recorder.path(),"new run overwrites old diagnostics");
    recorder.close();
    check(std::filesystem::file_size(first_path)>0,"old run was overwritten");
}

int main() {
    const auto root=std::filesystem::temp_directory_path()/(
        "awakening-diagnostics-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    try {
        raw_quaternion_and_protocol();
        split_and_coalesced_reads();
        camera_metadata_clone();
        asynchronous_recorder(root);
        std::filesystem::remove_all(root);
        std::cout<<"PASS: raw quaternion, CRC, chunk scanner, camera clone, async drain/overflow, run isolation\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<"FAIL: "<<error.what()<<'\n';
        std::filesystem::remove_all(root);
        return 1;
    }
}
