# Changelog

本文件记录项目的所有变更，按时间倒序排列。

格式基于 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循 [Semantic Versioning](https://semver.org/lang/zh-CN/)。

---

## [Unreleased]

### Added

- 新增架构决策记录 `docs/spec/mcl_adr_precharge.md`：自举电容预充电下沉到 mcl（方案 A：扩展 `mcl_hal_ops` 新增可选回调 `precharge_bootstrap`）。当前仅定案设计，暂不实现。

