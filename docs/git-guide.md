# Git 使用说明（记录改动 / 回档）

本目录已经是 git 仓库，远程在 `git@github.com:sheep5254/awake1.0.git`，分支 `main`。
下面是最常用的命令，按"我要干什么"查。

---

## 0. 先看现在的状态

```bash
git status          # 哪些文件改了/新增了
git log --oneline   # 提交历史（一行一个版本）
git tag             # 所有标签（备过份的版本）
```

## 1. 记录一次改动（最常用）

```bash
./script/git_snapshot.sh "把 pitch 低通撤掉，恢复增量限幅 3 度"
```

它会自动：暂存所有改动 → 提交 → 打一个 `snap-年月日_时分秒` 的标签。

想同时推到 GitHub（多一份异地备份）：

```bash
./script/git_snapshot.sh "说明这次改了什么" --push
```

手动做同样的事也行：

```bash
git add -A
git commit -m "说明这次改了什么"
git tag good-2026-10-20            # 给"确认好用"的版本起名字
git push origin main --tags        # 推上去
```

> 建议：**每次确认"这版能跑/打得好"就打个 `good-` 标签**，改坏了直接回这个标签。

## 2. 回档

### 整个仓库回到某个版本

```bash
git tag                         # 先看有哪些标签
git reset --hard good-2026-10-20_1321
```

> `reset --hard` 会丢掉当前未提交的改动。不确定时先 `./script/git_snapshot.sh "回档前先存一下"`。

### 只把某个文件恢复到某个版本

```bash
git checkout good-2026-10-20_1321 -- src/runtime/standard.cpp
```

（不影响其他文件，改完记得再提交一次）

### 撤销还没提交的改动

```bash
git checkout -- .               # 全部撤销（危险，不可恢复）
git checkout -- 某个文件         # 只撤销一个文件
```

### 回错了 / 想找回"被 reset 掉"的版本

```bash
git reflog                      # 所有 HEAD 移动记录，找到目标 commit
git reset --hard <commit>
```

## 3. 看"我改了什么"

```bash
git diff                        # 未提交的改动
git diff HEAD~1                 # 和上一个版本比
git show <commit>               # 某个版本改了什么
git log -p -- src/runtime/      # 某个目录的改动历史
```

## 4. 哪些东西不会进仓库（`.gitignore`）

| 目录 | 原因 |
|---|---|
| `build/` `bin/` | 编译产物，`ninja -C build standard` 能重新生成 |
| `log/` `record/` | 每次运行都重写，体积大（单次 jsonl 几十 MB） |
| `MvSdkLog/` | 相机 SDK 的运行日志 |
| `.backup_*/`、`.dsh-code-index/`、`*.avi` | 本地备份/工具缓存/录屏 |

也就是说：**代码、配置、模型、文档都在版本管理里**，跑出来的数据不会污染仓库。
如果某个新文件确实需要入库，`git add -f 文件` 可以强制加入。

## 5. 常用标签约定

| 前缀 | 含义 |
|---|---|
| `good-*` | 确认好用的版本，比赛/测试就用它 |
| `snap-*` | 日常自动存档（脚本生成） |
| `backup-*` | 大改之前的备份 |

## 6. 和 GitHub 同步

```bash
git push origin main --tags     # 上传
git pull origin main            # 下载别人/别的机器的改动
```

远程仓库只用于备份和换机器，**比赛现场断网也能用本地提交和回档**。
