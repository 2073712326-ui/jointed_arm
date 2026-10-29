# 中期项目学生工程

- [学生工程入口](student/README.md)
- [项目任务书](student/doc/任务书.md)
- [学生工程源码包](Double_jointed_robotic_arm_template.zip)

学生工程以当前完整工程 `52ec909` 为基础，机械臂管理、运动学、轨迹规划和 `robotControl()` 均留为空实现，保留类和函数接口。学生根据任务书自行组织代码，原工程作为教师参考。

任务书参考飞书[大项目说明](https://jiaoloong.feishu.cn/wiki/BQPfwJtEeikf7sk8fmtcudhvnyb)的行文方式，按项目内容、要求、评价与调优方向组织。使用 `shuorenhua` 整理措辞，将程序中的失败、取消和掉线行为写成具体要求。

## 交付检查

2026-10-24 完成以下检查：

| 检查 | 结果 |
|---|---|
| F407 GCC | 编译链接成功，生成 ELF、HEX、BIN；有工程原有的系统调用桩及 RWX 段警告 |
| F407 Keil / ARM Compiler 6.23 | 完整重建，0 错误、0 警告 |

构建日志保存在原工程的 `build/student-gcc.log`、`build/student-keil.log`。学生源码包包含工程源码、依赖库和任务书。

实机验收由学生按任务书完成。本次没有烧录或运行机械臂。
