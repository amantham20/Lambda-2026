# Network Allowlist Request (Team 5509 – Lambda-2026)

Purpose: ask IT/administrators to unblock the public hosts needed to build and
deploy this robot project and to use AI coding assistants (Claude Code, ChatGPT).

Sources: this repo's `build.gradle`, `settings.gradle`, `gradle-wrapper.properties`,
`vendordeps/*.json`, `.github/workflows/build.yml`; Anthropic's
[Claude Code network requirements](https://code.claude.com/docs/en/network-config);
OpenAI's [ChatGPT network recommendations](https://help.openai.com/en/articles/9247338-network-recommendations-for-chatgpt-errors-on-web-and-apps).

Confidence: **[repo]** = read directly from this repo, **[vendor docs]** = from the
vendor's published docs, **[verify]** = standard for the tool but not read from this
repo; confirm with a proxy log (see the end of this file).

All traffic is HTTPS (443). No inbound access is required.

---

## 1. This repository

### 1a. Toolchain: pinned versions [repo]

| Component | Version | Source |
|---|---|---|
| Java (JDK) | 17 | `build.gradle`, CI |
| Gradle | 8.11 | `gradle-wrapper.properties` (`services.gradle.org`) |
| GradleRIO plugin (WPILib) | 2026.2.1 | `build.gradle` |
| JUnit Jupiter | 5.10.1 | `build.gradle` |
| Team number / year | 5509 / 2026 | `.wpilib/wpilib_preferences.json` |

### 1b. Vendor libraries (`vendordeps/`) [repo]

| Library | Version | Maven artifacts (group:artifact) | Maven host |
|---|---|---|---|
| WPILib (core + New Commands) | 2026.2.1 | `edu.wpi.first.*` | `frcmaven.wpi.edu` |
| CTRE Phoenix 6 | 26.1.1 | `com.ctre.phoenix6:wpiapi-java`, `api-cpp`, `tools`, `com.ctre.phoenix6.sim:*` | `maven.ctr-electronics.com` |
| PathPlannerLib | 2026.1.2 | `com.pathplanner.lib:PathplannerLib-java`, `-cpp` | `3015rangerrobotics.github.io` |
| PhotonLib (PhotonVision) | v2026.2.2 | `org.photonvision:photonlib-java`, `photontargeting-java`, `-cpp` | `maven.photonvision.org` |
| AdvantageKit | 26.0.0 | `org.littletonrobotics.akit:akit-java`, `akit-wpilibio` | `frcmaven.wpi.edu` (littletonrobotics repo) |

Java packages actually imported by `src/`: `edu.wpi.first.*`, `com.ctre.phoenix6.*`,
`com.pathplanner.lib.*`, `org.photonvision.*`, `org.littletonrobotics.junction.*`.

### 1c. Hosts to allow for building (Gradle/GradleRIO)

| Host | Why | Confidence |
|---|---|---|
| `services.gradle.org` | Downloads the Gradle 8.11 wrapper distribution | repo |
| `plugins.gradle.org` | Resolves the `edu.wpi.first.GradleRIO` plugin | repo (`gradlePluginPortal()`) |
| `frcmaven.wpi.edu` | WPILib libraries, native/JNI libs, AdvantageKit mirror | repo + vendor docs |
| `maven.ctr-electronics.com` | CTRE Phoenix 6 | repo |
| `3015rangerrobotics.github.io` | PathPlannerLib maven repo and vendordep JSON | repo |
| `maven.photonvision.org` | PhotonLib | repo |
| `repo.maven.apache.org` (Maven Central) | JUnit 5 and transitive libraries (Jackson, EJML, etc. used by WPILib) | verify |
| `repo1.maven.org` | Maven Central alias some tools redirect to | verify |
| `github.com`, `objects.githubusercontent.com`, `raw.githubusercontent.com`, `api.github.com` | Source hosting, vendordep "latest" JSON URLs (AdvantageKit), WPILib installer and tool releases, GitHub Actions | repo + verify |

Vendordep update URLs (only needed when updating libraries in VS Code):
- `https://github.com/Mechanical-Advantage/AdvantageKit/releases/latest/download/AdvantageKit.json`
- `https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json`
- `https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2026-latest.json`
- `https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json`

### 1d. CI (`.github/workflows/build.yml`) [repo]

Runs on GitHub-hosted runners, so nothing needs unblocking on school machines. For
reference it uses `actions/checkout@v4`, `actions/setup-java@v4` (Temurin 17),
`gradle/actions/setup-gradle@v3`, `actions/upload-artifact@v4`, and the same Maven hosts as 1c.

### 1e. Developer tools used by the team (not in the repo build, but needed to work)

| Tool | Host(s) | Confidence |
|---|---|---|
| WPILib installer / VS Code bundle | `github.com/wpilibsuite/allwpilib/releases` (+ `objects.githubusercontent.com`) | vendor docs |
| VS Code + Java/WPILib extensions | `marketplace.visualstudio.com`, `*.vo.msecnd.net`, `update.code.visualstudio.com`, `open-vsx.org` (if used) | verify |
| PathPlanner app | `github.com/mjansen4857/pathplanner/releases` (also Microsoft Store) | verify |
| Phoenix Tuner X | Microsoft Store / app stores; `ctr-electronics.com` | vendor docs |
| PhotonVision (coprocessor image / client) | `github.com/PhotonVision/photonvision/releases`, `docs.photonvision.org` | verify |
| AdvantageScope | `github.com/Mechanical-Advantage/AdvantageScope/releases` | verify |
| Driver Station / roboRIO imaging | NI downloads: `ni.com` (offline installers also exist) | vendor docs |
| roboRIO deploy | LAN only (USB/Ethernet/Wi-Fi to the robot, `10.55.9.2` / `roborio-5509-frc.local`, ports 22, 1735, 5800-5810). Not internet. | vendor docs |

---

## 2. Claude Code / Claude (Anthropic)

From Anthropic's official list (fetched 2026-10-09):

| Host | Required for | Required? |
|---|---|---|
| `api.anthropic.com` | Model/API requests, feature flags | **Required** |
| `claude.ai` | Account sign-in | **Required** |
| `claude.com` | Sign-in redirect, docs lookups | **Required** |
| `platform.claude.com` | Console auth, OAuth token exchange/refresh | **Required** |
| `downloads.claude.ai` | Native installer, auto-update, plugin downloads | Required for native install |
| `registry.npmjs.org` | npm install of Claude Code, plugins, npx-based MCP servers | Required for npm install |
| `github.com` | Plugin marketplaces; cloud sessions clone repos | Needed for plugins and for Claude Code on the web |
| `mcp-proxy.anthropic.com` | claude.ai connectors (MCP) | Optional |
| `storage.googleapis.com` | Plugin metadata (and installer on versions older than 2.1.116) | Optional |
| `code.claude.com` | Documentation lookups | Optional |
| `raw.githubusercontent.com` | `/release-notes` changelog | Optional |
| `bridge.claudeusercontent.com` | Claude in Chrome extension | Optional |
| `*.frame.claudeusercontent.com` | Artifacts | Optional |
| `assets-proxy.anthropic.com`, other `*.claudeusercontent.com`, `*.claudemcpcontent.com` | claude.ai / Desktop app UI, artifacts, MCP widgets | Required for claude.ai web/Desktop |
| `http-intake.logs.us5.datadoghq.com`, `browser-intake-us5-datadoghq.com` | Operational telemetry | Optional; admins can block and set `CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC=1` |
| `formulae.brew.sh` | Homebrew update checks | Only for Homebrew installs |
| `fonts.googleapis.com`, `fonts.gstatic.com`, `cdnjs.cloudflare.com`, `cdn.jsdelivr.net`, `unpkg.com`, `cdn.tailwindcss.com`, `code.jquery.com` | Fonts/libraries inside artifacts | Optional |

Minimum for a student to sign in and use Claude Code via the CLI:
`api.anthropic.com`, `claude.ai`, `claude.com`, `platform.claude.com`, plus
`downloads.claude.ai` (or `registry.npmjs.org`) to install.
For claude.ai in a browser also add the `*.claudeusercontent.com` / `assets-proxy.anthropic.com` hosts.

Notes for admins: proxies are supported via `HTTPS_PROXY`; SOCKS is not supported;
corporate TLS-inspection CAs are trusted if installed in the OS store (or via `NODE_EXTRA_CA_CERTS`).

Claude Code on the web (cloud sessions) additionally needs the Claude GitHub App
installed on the team's GitHub org/repo.

---

## 3. ChatGPT / OpenAI

OpenAI's published guidance (help article 9247338) names:

| Host | Purpose |
|---|---|
| `*.chatgpt.com` (incl. `chatgpt.com`) | Web app and API |
| `ws.chatgpt.com` | WebSocket streaming (allow WebSocket upgrades on TCP 443; TLS inspection can break long-lived sockets) |
| `*.auth.openai.com` | Sign-in |
| `*.oaiusercontent.com` (e.g. `files.oaiusercontent.com`) | File uploads/downloads |
| UDP 3478 + OpenAI-published IP ranges | ChatGPT Voice only (optional) |

Commonly also needed [verify, not in the article I could read]: `openai.com`,
`chat.openai.com` (legacy redirect), `cdn.oaistatic.com` / `*.oaistatic.com` (static assets),
`api.openai.com` (API / SDK users), `registry.npmjs.org` and `github.com` (Codex CLI install).
Ask IT to run the browser dev tools Network tab on first load to confirm.

---

## 4. How to confirm the list empirically

1. On an unrestricted network: `./gradlew build --refresh-dependencies` through a logging
   proxy (mitmproxy/Squid) with `HTTPS_PROXY` set, and sort unique hosts.
2. For Claude Code: `claude --debug`, plus `/status` to view proxy state.
3. For ChatGPT: browser DevTools -> Network -> "Domain" column.
4. Alternative if allowlisting is hard: build once on an open network and copy `~/.gradle`
   and `~/wpilib/2026` to school machines (offline mode: `./gradlew build --offline`).
