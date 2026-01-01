// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive course for senior undergraduate AI students',
  favicon: 'img/favicon.ico',

  /* =========================
     DEPLOYMENT (VERCEL)
     ========================= */
  url: 'https://hackathon-quarter4-alpha.vercel.app',
  baseUrl: '/',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  trailingSlash: false,

  /* =========================
     HEAD TAGS
     ========================= */
  headTags: [
    {
      tagName: 'link',
      attributes: {
        rel: 'stylesheet',
        href: 'https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.5.1/css/all.min.css',
        crossorigin: 'anonymous',
        referrerpolicy: 'no-referrer',
      },
    },
  ],

  /* =========================
     I18N
     ========================= */
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  /* =========================
     PRESETS
     ========================= */
  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  /* =========================
     THEME CONFIG
     ========================= */
  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',

    navbar: {
      title: 'Physical AI',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.png',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'docs',
          label: 'Book',
          position: 'left',
        },
        {
          href: 'https://github.com/Anas-Rajput12/Hackathon-quarter4',
          label: 'GitHub',
          position: 'right',
          className: 'navbar-github-link',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Course',
          items: [{ label: 'Book', to: '/docs/intro' }],
        },
      ],
      copyright: `© ${new Date().getFullYear()} Physical AI & Humanoid Robotics`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },
};

export default config;
