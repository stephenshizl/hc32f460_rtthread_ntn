# youyoutech Project Rules / youyoutech 工程规范

## What this is / 这是什么

- Rules for developers and tools working in this repository. / 约束在本仓库内工作的开发人员与工具。
- Core principle: make only minimal, task-focused, verifiable changes. / 核心原则：只做最小化、面向任务、可验证的改动。

## Allowed / 可以做什么

- Read files directly related to the task. / 阅读与任务直接相关的文件。
- Make minimal required edits, and update directly related docs or tests when needed. / 仅做完成任务所需的最小改动，并在需要时同步更新直接相关的文档或测试。
- Use existing directories, scripts, and checks. / 复用现有目录、脚本和检查流程。

## Strictly forbidden / 严格禁止

- Unrelated refactors, bulk renames, repo-wide formatting, or unrelated file changes. / 与任务无关的重构、批量重命名、全仓格式化或其他无关改动。
- Generated artifacts, temporary files, logs, binaries, local environment files, or secrets. / 提交生成物、临时文件、日志、二进制文件、本地环境文件或任何密钥等敏感信息。
- Dependency churn, third-party/imported code changes, or weakened/removed tests unless the task explicitly requires them. / 除非任务明确要求，否则禁止随意变更依赖、修改第三方/导入代码，或弱化/删除测试。
- Creating Markdown documents without explicit instructions. / 没有明确指令时，禁止随意新建 Markdown 文档。
- Placing code in the wrong directory or creating new subdirectories under major directories without a clear requirement. / 禁止乱放代码，禁止在主要目录下无明确要求时新建子目录。
- Editing a BSP `rtconfig.h` by hand; generate it from `.config` with `scons --menuconfig` or `scons --pyconfig-silent`. / 禁止手工修改 BSP 的 `rtconfig.h`；应通过 `scons --menuconfig` 或 `scons --pyconfig-silent` 基于 `.config` 生成。
- History rewrites or overwriting others' work. / 改写历史或覆盖他人工作成果。

## Placement / 放置规则

- Use existing directories such as `bsp/`, `components/`, `documentation/`, `examples/`, `include/`, `libcpu/`, `src/`, and `tools/`. / 使用现有目录，如 `bsp/`、`components/`、`documentation/`、`examples/`、`include/`、`libcpu/`、`src/`、`tools/`。
- BSP code only in `bsp/`; kernel code in `src/`; tests/examples in `examples/`; component code in `components/`; abstracted driver code in `components/drivers/`; chip or architecture shared porting code in `libcpu/`. / 板级支持包代码只放在 `bsp/`；内核代码放在 `src/`；测试/例程代码放在 `examples/`；组件代码放在 `components/`；抽象层驱动相关代码放在 `components/drivers/`；芯片或架构公共移植代码放在 `libcpu/`。
- Keep repository-entry files such as `AGENTS.md`, `README.md`, and `LICENSE` in the root; put general documentation in `documentation/`. / `AGENTS.md`、`README.md`、`LICENSE` 等仓库入口文件放在根目录；通用文档放在 `documentation/`。

## Precedence / 优先级

- Development and review tools should also follow `.github/project-instructions.md`. / 开发和审查工具还应遵循 `.github/project-instructions.md`。
- If a more specific repository rule conflicts with this file, follow `.github/project-instructions.md`, `.github/CONTRIBUTING.md`, `documentation/7.contribution/coding_style_cn.md`, and `documentation/7.contribution/coding_style_en.md`. / 如与更具体的仓库规则冲突，以 `.github/project-instructions.md`、`.github/CONTRIBUTING.md`、`documentation/7.contribution/coding_style_cn.md`、`documentation/7.contribution/coding_style_en.md` 为准。

## 署名与交付 / Attribution and delivery

- 项目自有代码的公司署名统一使用 `youyoutech`。 / Use `youyoutech` for company attribution in project-owned code.
- 注释、文档、界面及提交说明不得添加生成工具、服务商及其产品名称作为作者、版权方或协作署名，也不添加生成来源说明。 / Do not add generation tools, service providers or their product names as authors, copyright owners, collaborators or generation-origin notices in comments, documentation, UI or commit messages.
- 保留第三方版权、许可证及真实贡献者信息，不将第三方成果改署为本公司。 / Preserve third-party copyright, licenses and real contributor credits; do not reattribute third-party work to the company.
- 清理说明文字不得改变功能相关的符号、外设型号、协议、接口、依赖或文件路径。 / Text cleanup must preserve functional symbols, peripheral models, protocols, interfaces, dependencies and paths.
