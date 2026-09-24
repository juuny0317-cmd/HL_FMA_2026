import { ExtensionContext } from "@foxglove/extension";

import { initDashboardPanel } from "./DashboardPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "HL KU Dashboard",
    initPanel: initDashboardPanel,
  });
}
