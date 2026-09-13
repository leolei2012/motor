# third_party

本目录存放第三方库（vendor，直接拷贝）。

## 库清单

| 目录     | 说明           |
| -------- | -------------- |
| `filter` | 滤波（EMA 等） |
| `pid`    | PID 控制器     |

## 约定

- 第三方库源码不改，需要扩展时在外部包一层（见 `docs/规范/coding_standard.md` §9.3）。
- 待有独立远端仓库后，可按需迁移为 git submodule。
