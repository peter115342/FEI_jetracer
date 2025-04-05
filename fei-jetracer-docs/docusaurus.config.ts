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
  organizationName: "peter115342",
  projectName: "FEI_jetracer",
  trailingSlash: true,
  deploymentBranch: "gh-pages",

  onBrokenLinks: "warn",
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
          title: "Faculty",
          items: [
            {
              label: "FEI STU",
              href: "https://www.fei.stuba.sk/",
            },
            {
              label: "Ústav automobilovej mechatroniky FEI STU",
              href: "http://uamt.fei.stuba.sk/web/?q=sk",
            },
          ],
        },
        {
          title: "References",
          items: [
            {
              label: "Waveshare",
              href: "https://www.waveshare.com/wiki/JetRacer_ROS_AI_Kit",
            },
            {
              label: "DonkeyCar",
              href: "https://docs.donkeycar.com/",
            },
            {
              label: "YOLO",
              href: "https://pjreddie.com/darknet/yolo/",
            },
          ],
        },
        {
          title: "Github",
          items: [
            {
              label: "This Project",
              href: "https://github.com/peter115342/FEI_jetracer",
            },
            {
              label: "peter115342",
              href: "https://github.com/peter115342",
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
