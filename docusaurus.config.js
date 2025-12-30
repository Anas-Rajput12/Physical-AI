// @ts-check
require('dotenv').config();
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive course for senior undergraduate AI students',
  favicon: 'img/favicon.ico',

  // ✅ Correct GitHub Pages URL
  url: 'https://anas-rajput12.github.io',

  // ✅ Repo name (must match exactly)
  baseUrl: '/AI-Hackathon/',

  // ✅ GitHub Pages deployment config
  organizationName: 'Anas-Rajput12',
  projectName: 'AI-Hackathon',
  deploymentBranch: 'gh-pages',

  // ✅ Strict but correct
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  scripts: [
    {
      src: '/js/language-switcher.js',
      async: true,
    },
  ],

  headTags: [
    {
      tagName: 'link',
      attributes: {
        rel: 'stylesheet',
        href: 'https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css',
        crossorigin: 'anonymous',
      },
    },
    {
      tagName: 'script',
      innerHTML: `
        document.addEventListener('DOMContentLoaded', function() {
          const languageSwitcher = document.getElementById('language-switcher');
          if (languageSwitcher) {
            const savedLanguage = localStorage.getItem('preferredLanguage') || 'en';
            languageSwitcher.value = savedLanguage;

            languageSwitcher.addEventListener('change', function() {
              localStorage.setItem('preferredLanguage', this.value);
              window.dispatchEvent(new CustomEvent('languageChanged', {
                detail: { language: this.value }
              }));
            });
          }
        });
      `,
    },
  ],

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',

          // ✅ Docs live at site root
          routeBasePath: '/',

          editUrl: 'https://github.com/Anas-Rajput12/AI-Hackathon',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

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
              style="padding:4px;border-radius:4px;border:1px solid #ccc;">
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
          items: [
            {
              label: 'Book',
              to: '/intro', // ✅ correct path
            },
          ],
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
