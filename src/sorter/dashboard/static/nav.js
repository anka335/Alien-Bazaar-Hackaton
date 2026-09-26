// Page switcher (Dashboard / Manual / Twin), pinned to the bottom-left corner. Skipped inside
// an iframe, e.g. the twin embedded in the dashboard.
(() => {
  if (window.top !== window) return;
  const pages = [["/", "Dashboard"], ["/manual", "Manual"], ["/twin", "Twin"]];
  const nav = document.createElement("nav");
  nav.setAttribute("aria-label", "Pages");
  nav.style.cssText =
    "position:fixed;left:12px;bottom:12px;z-index:1000;display:flex;gap:4px;padding:4px;" +
    "background:rgb(20 28 38 / 0.85);border-radius:8px;font:600 13px system-ui,sans-serif";
  for (const [href, label] of pages) {
    const a = document.createElement("a");
    a.href = href;
    a.textContent = label;
    const here = location.pathname === href;
    a.style.cssText =
      "padding:6px 10px;border-radius:6px;text-decoration:none;" +
      (here ? "background:#3cc8e2;color:#0b1118" : "color:#cfe3ea");
    if (here) a.setAttribute("aria-current", "page");
    nav.append(a);
  }
  document.body.append(nav);
})();
