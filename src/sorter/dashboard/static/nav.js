// Page switcher (Dashboard / Manual / Calibrate / Twin), pinned to the bottom-left corner. Skipped inside
// an iframe, e.g. the twin embedded in the dashboard.
// One server runs either the sorter (`python -m sorter run`) or manual control (`… manual`):
// /api/manual answers only in the manual mode, and the page of the other mode is greyed out.
(() => {
  if (window.top !== window) return;
  const pages = [
    ["/", "Dashboard", "run", "The dashboard needs the sorter: python -m sorter run"],
    ["/manual", "Manual", "manual", "Manual control needs: python -m sorter manual"],
    ["/calibrate", "Calibrate", "manual", "Calibration needs: python -m sorter manual"],
    ["/twin", "Twin", null, ""],
  ];
  const nav = document.createElement("nav");
  nav.setAttribute("aria-label", "Pages");
  nav.style.cssText =
    "position:fixed;left:12px;bottom:12px;z-index:1000;display:flex;gap:4px;padding:4px;" +
    "background:rgb(20 28 38 / 0.85);border-radius:8px;font:600 13px system-ui,sans-serif";
  const links = [];
  for (const [href, label, mode, hint] of pages) {
    const a = document.createElement("a");
    a.href = href;
    a.textContent = label;
    const here = location.pathname === href;
    a.style.cssText =
      "padding:6px 10px;border-radius:6px;text-decoration:none;" +
      (here ? "background:#3cc8e2;color:#0b1118" : "color:#cfe3ea");
    if (here) a.setAttribute("aria-current", "page");
    nav.append(a);
    links.push({ a, mode, hint, here });
  }
  document.body.append(nav);

  fetch("/api/manual", { cache: "no-store" })
    .then((r) => (r.ok ? "manual" : r.status === 404 ? "run" : null))
    .catch(() => null)
    .then((mode) => {
      if (!mode) return;
      for (const { a, mode: m, hint, here } of links) {
        if (!m || m === mode || here) continue;
        a.removeAttribute("href");
        a.setAttribute("aria-disabled", "true");
        a.title = hint;
        a.style.opacity = "0.4";
        a.style.cursor = "not-allowed";
      }
    });
})();
