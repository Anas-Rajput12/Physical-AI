// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';
import 'dotenv/config';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive course for senior undergraduate AI students',
  favicon: 'img/favicon.ico',

  /* =========================
     VERCEL DEPLOYMENT CONFIG
     ========================= */
  url: 'https://ai-hackathon-vqou.vercel.app',
  baseUrl: '/', // MUST be '/' for Vercel root deploy

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  trailingSlash: false,

  /* =========================
     SCRIPTS & HEAD TAGS
     ========================= */
  scripts: [
    {
      src: 'js/language-switcher.js', // relative path (baseUrl-safe)
      async: true,
    },
  ],

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
    {
      tagName: 'script',
      innerHTML: `
        document.addEventListener('DOMContentLoaded', () => {
          const switcher = document.getElementById('language-switcher');
          if (!switcher) return;

          const saved = localStorage.getItem('preferredLanguage') || 'en';
          switcher.value = saved;

          switcher.addEventListener('change', (e) => {
            const lang = e.target.value;
            localStorage.setItem('preferredLanguage', lang);
            window.dispatchEvent(
              new CustomEvent('languageChanged', { detail: { language: lang } })
            );
          });
        });
      `,
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
          editUrl: 'https://github.com/Anas-Rajput12/AI-Hackathon',
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
        src: 'https://imgcdn.stablediffusionweb.com/2024/2/29/4271b7e7-6e2b-498a-ae2c-89a709c2e175.jpg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'docs',
          label: 'Book',
        },

        {
          type: 'html',
          position: 'right',
          value: `
            <select id="language-switcher"
              style="
                padding: 0.3rem 0.6rem;
                border-radius: 6px;
                border: 1px solid var(--ifm-color-emphasis-300);
                background: var(--ifm-background-surface-color);
                color: var(--ifm-font-color-base);
              ">
              <option value="en">English</option>
              <option value="ur">Urdu</option>
              <option value="sd">Sindhi</option>
            </select>
          `,
        },

        {
          href: 'https://github.com/Anas-Rajput12/AI-Hackathon',
          label: 'GitHub',
          position: 'right',
        },

        {
          type: 'dropdown',
          label: 'Account',
          position: 'right',
          items: [
            { label: 'Sign In', to: '/signin' },
            { label: 'Sign Up', to: '/signup' },
          ],
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
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Anas-Rajput12/AI-Hackathon',
            },
          ],
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
