import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

const sidebars: SidebarsConfig = {
  // explicit order
  tutorialSidebar: [
    "intro",
    "assembly/assembly",
    {
      type: "category",
      label: "Installation",
      items: [
        "installation/installation_jetson",
        "installation/installation_host",
      ],
    },
    "communication/communication",
    "movement-control/movement_control",
    "node-topics/node_topics",
    {
      type: "category",
      label: "FEIcar",
      items: ["FEIcar/FEIcar_installation", "FEIcar/object_detection"],
    },
  ],
};

export default sidebars;
