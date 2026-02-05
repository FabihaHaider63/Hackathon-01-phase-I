export default {
  title: "AI-Driven Book on ROS 2",
  tagline: "An educational resource on Physical AI and ROS 2",
  favicon: "img/favicon.ico",
  url: "https://hackathon-01-phase-i.vercel.app",
  baseUrl: "/",
  organizationName: "mr-computers",
  projectName: "ai-book-docusaurus",
  i18n: {
    defaultLocale: "en",
    locales: ["en"]
  },
  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: require.resolve("./sidebars.js"),
          editUrl: "https://github.com/FabihaHaider63/Hackathon-01-phase-I/edit/main/docs/"
        },
        blog: false,
        theme: {
          customCss: require.resolve("./src/css/custom.css")
        }
      }
    ]
  ],
  themeConfig: {
    colorMode: { defaultMode: "light", disableSwitch: true },
    navbar: {
      title: "Physical AI & Humanoid Robotics",
      items: [
        { type: "docSidebar", sidebarId: "tutorialSidebar", position: "left", label: "Textbook" }
      ]
    }
  },
  staticDirectories: ["static"]
};
