import { StrictMode, useEffect } from "react";
import { createRoot } from "react-dom/client";
import { Toasts } from "./components/common";
import { TopBar, currentTab } from "./components/TopBar";
import { usePath } from "./hooks";
import { AutoPage } from "./pages/AutoPage";
import { CalibratePage } from "./pages/CalibratePage";
import { ManualPage } from "./pages/ManualPage";
import { RoverPage } from "./pages/RoverPage";
import { TwinPage } from "./pages/TwinPage";
import { SorterProvider, useSorter } from "./sorter";
import "./styles/base.css";
import "./styles/auto.css";
import "./styles/setup.css";
import "./styles/calibrate.css";
import "./styles/twin.css";
import "./styles/rover.css";

const PAGES: Record<string, () => React.JSX.Element> = {
  "/load": AutoPage,
  "/unload": AutoPage,
  "/manual": ManualPage,
  "/calibrate": CalibratePage,
  "/3d": TwinPage,
  "/rover": RoverPage,
};

function App(): React.JSX.Element {
  const path = usePath();
  const { status, hold } = useSorter();
  // Space or Esc anywhere holds the arm. preventDefault keeps Space from also clicking a focused button.
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.code === "Space" || e.key === "Escape") {
        e.preventDefault();
        hold();
      }
    };
    document.addEventListener("keydown", onKey);
    return () => document.removeEventListener("keydown", onKey);
  }, [hold]);
  const Page = PAGES[currentTab(path, status?.operator)];
  return (
    <>
      <TopBar />
      <Page />
      <Toasts />
    </>
  );
}

const root = document.getElementById("root");
if (!root) throw new Error("no #root");
createRoot(root).render(
  <StrictMode>
    <SorterProvider>
      <App />
    </SorterProvider>
  </StrictMode>,
);
