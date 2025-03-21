import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";

const config: Config = {
  title: "FEI JetRacer",
  tagline: "AI-powered racing robot based on NVIDIA Jetson Nano",
  favicon: "img/favicon.ico",

  // Set the production url of your site here
  url: "https://peter115342.github.io",
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: "/FEI_jetracer/",

  // GitHub pages deployment config
  organizationName: "peter115342", // Usually your GitHub org/user name
  projectName: "FEI_jetracer", // Usually your repo name
  trailingSlash: true,
  deploymentBranch: "gh-pages",

  onBrokenLinks: "throw",
  onBrokenMarkdownLinks: "warn",

  // Internationalization configuration
  i18n: {
    defaultLocale: "en",
    locales: ["en", "sk"],
    localeConfigs: {
      en: {
        htmlLang: "en-US",
        label: "English",
      },
      sk: {
        htmlLang: "sk-SK",
        label: "Slovenčina",
      },
    },
  },

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          editUrl:
            "https://github.com/peter115342/FEI_jetracer/tree/main/fei-jetracer-docs/",
        },
        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: "img/jetracer-social-card.jpg",
    navbar: {
      title: "FEI JetRacer",
      logo: {
        alt: "FEI STU Logo",
        src: "img/STU-FEI-nvf.png",
      },
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar",
          position: "left",
          label: "Tutorials",
        },

        {
          type: "localeDropdown",
          position: "right",
        },
        {
          href: "https://github.com/peter115342/FEI_jetracer",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Docs",
          items: [
            {
              label: "Introduction",
              to: "/docs/intro",
            },
            {
              label: "Assembly",
              to: "/docs/assembly",
            },
            {
              label: "Movement Control",
              to: "/docs/movement-control",
            },
          ],
        },
        {
          title: "References",
          items: [
            {
              label: "FEI STU",
              href: "https://www.fei.stuba.sk/",
            },
            {
              label: "Waveshare",
              href: "https://www.waveshare.com/wiki/JetRacer_ROS_AI_Kit",
            },
            {
              label: "Ústav automobilovej mechatroniky FEI STU",
              href: "http://uamt.fei.stuba.sk/web/?q=sk",
            },
          ],
        },
        {
          title: "More",
          items: [
            {
              label: "GitHub",
              href: "https://github.com/peter115342/FEI_jetracer",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} FEI STU JetRacer Project. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
