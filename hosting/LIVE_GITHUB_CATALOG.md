# Alfred Robot Player — Live GitHub catalog for the existing ChatGPT Site

**Identity:** Keep the existing private `Alfred Robot Player` ChatGPT Site,
`https://alfred-robot-player.curlyeave.chatgpt.site`. Do **not** create a
new Site. This source code is a **one-time upgrade**, not a claim that the
currently published private Site has already been updated.

The old Sites export (PR #92) embeds a static `docs/` library and checks
`hosted-version.json`; that mechanism cannot discover new recordings.
This change retains the same user interface, pages, Three.js vendor files and
any existing video media, but adds a browser-side bridge to the GitHub
catalogue (starting at revision **36**, 57 entries including MicroDuck).

## Data flow after the one-time Site deployment

1. Open any page of the existing Site. `github-live-boot.js` registers
   `github-live-sw.js` on the same origin (`scope: /`) and claims this tab.
2. The existing library's initial load or **Проверить обновления** gets
   `/hexapod/library.json`. The worker asks the GitHub API for the head SHA
   of `Dmitry-dev-pet/alfred-hexapod-lab` branch
   `feature/android-local-offline-viewer-20261104`.
3. It downloads the library at that **immutable commit**, verifies the schema,
   ID/path/size/SHA-256 allowlist, and remembers the verified catalog.
4. Clicking any entry navigates to its existing same-origin
   `/hexapod/<id>/` path. If the HTML or referenced JS/JSON/CSS is listed
   in the trusted catalog, the worker downloads the *exact* GitHub blob,
   verifies byte count and SHA-256, and serves it with the correct browser
   MIME type. No new HTML/JS template is required on ChatGPT Sites.
5. Existing catalog entries and locally published assets remain available;
   unlisted site routes, `/api/`, other Sites apps, existing CSS and vendor
   libraries are never proxied by this worker. Original third-party STL models
   remain at upstream pinned URLs and preserve existing SHA verification.
6. After a GitHub outage, the last browser-verified catalog and any previously
   viewed assets remain available locally; the UI labels this a cached copy.

**New experiment pipeline:** Commit finished physics replay page/assets to
the owner-managed **working player branch**, add a properly checked new
entry to `docs/hexapod/library.json` using
`scripts/register_viewer_replays.py`, and merge into that working branch.
On the next site open / library refresh, the new recording appears. No
ChatGPT Site version or additional infrastructure is needed.

MicroDuck: `/hexapod/microduck-standup-v1/`, one entry with seven six-second
3D cases, 38 original STL references, 70 visual parts, 300 MuJoCo frames per
case. Source PR #98 was merged into the player branch; its replay is research
only and does **not** establish hardware sim-to-real reliability.

## One-time publishing to the **existing** Sites project

1. Open the existing *Alfred Robot Player* Site project in **ChatGPT Sites** or
   Work. Keep its identity, access settings, address, vendor Three.js bundle,
   verified media and existing source repository intact.
2. Bring in the updated source from
   `feature/sites-private-player-20261107` (after this feature is integrated
   into that branch), including `hosting/github-live-sw.js`,
   `hosting/github-live-boot.js` and `hosting/player-refresh-live.js`.
3. Rebuild with `scripts/build_hosted_player.py --site <existing-site-root>
   --vendor <existing-verified-three> --media <existing-verified-media>`
   and verify the output `dist/github-live-sw.js`,
   `dist/github-live-boot.js`, `dist/hexapod/player-shell.js`.
   The original videos are retained by reusing the existing `--media`;
   do not silently omit that argument when the current Site uses media.
4. Save/preview the **existing** Site; check its browser support for service
   workers, allowed external GitHub fetches/CORS and WebGL. Deploy *one*
   update to the **same** Site URL, not a replacement Site.
5. On the published Site, open the bottom-sheet library and
   **Проверить обновления**; expect at least revision 36 and
   **MicroDuck · 7 испытаний вставания**. Open the MicroDuck entry and
   verify actual original STL 3D rendering, controls, mobile touch and
   all seven scenario transitions.

The service-worker route is browser capability dependent. If ChatGPT Sites
disallows registration or GitHub CORS via its hosting/CSP, the last bundled
catalog remains available; an explicit Site Worker/server-side alternative
will be necessary. Do not claim production-ready until a real signed-in
Sites-browser test passes.

## Security / integration contract

- This is strictly **read-only**. No GitHub tokens or private research
  resources are embedded in browser code.
- Only the exact public repository and branch are read. GitHub API returns a
  fixed 40-character SHA; all file requests use that SHA rather than a mutable
  `main` or branch URL.
- Only catalogued HTML/JS/JSON/CSS with correct size, path and SHA-256 execute.
  The trusted GitHub branch is part of the web application's code supply
  chain. Protect its write permissions and review new replay scripts.
- Browser-cache fallback is explicitly marked; an unverified response or
  broken digest fails closed, not substituted with an imaginary robot.
- This Site bridge does not install a background daemon on the tablet, does
  not require Termux, and does not alter `main`.
- Third-party STL rights remain unchanged; do not copy vendor STL or training
  weights into the Sites build without separately reviewing licence scope.

## Tests

```bash
node --test tests/test_sites_live_worker.mjs
python -m unittest discover -s tests -p 'test_hosted_player.py' -v
```

CI also performs an offline integrity-matched *disposable* Site export and
checks the new boot/SW scripts are in the published `dist/`. This is not
a substitute for device and real ChatGPT Sites browser testing.
