// @ts-check
require('dotenv').config();
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive course for senior undergraduate AI students',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-username.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages, this is usually your repo name
  baseUrl: '/my-physical-ai-book',

  // GitHub pages deployment config
  organizationName: 'your-username', // Usually your GitHub org/user name
  projectName: 'my-physical-ai-book', // Usually your repo name
  deploymentBranch: 'gh-pages', // Branch that GitHub Pages will deploy from

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Add Font Awesome for GitHub icon
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
        integrity: 'sha512-9usAa10IRO0HhonpyAIVpjrylPvoDwiPUiKdWk5t3PyolY1cOd4DSE0Ga+ri4AuTroPR5aQvXU9xC6qOPnzFeg==',
        crossorigin: 'anonymous',
      },
    },
    {
      tagName: 'script',
      attributes: {},
      innerHTML: `
        document.addEventListener('DOMContentLoaded', function() {
          const languageSwitcher = document.getElementById('language-switcher');
          if (languageSwitcher) {
            // Set initial value from localStorage
            const savedLanguage = localStorage.getItem('preferredLanguage') || 'en';
            languageSwitcher.value = savedLanguage;

            languageSwitcher.addEventListener('change', function() {
              const selectedLanguage = this.value;
              localStorage.setItem('preferredLanguage', selectedLanguage);

              // Dispatch a custom event to notify other parts of the app
              window.dispatchEvent(new CustomEvent('languageChanged', {
                detail: { language: selectedLanguage }
              }));
            });
          }
        });
      `,
    }
  ],

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Set to false to disable the docs plugin
          routeBasePath: '/',
          editUrl: 'https://github.com/Anas-Rajput12/Physical-AI',
        },
        blog: false, // Disable blog for this educational book
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  plugins: [
    // Plugin to inject environment variables
    // Temporarily removed @docusaurus/plugin-client-redirects due to compatibility issue
    // [
    //   '@docusaurus/plugin-client-redirects',
    //   {
    //     // Optional: if you need client-side redirects
    //   },
    // ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
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
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            type: 'html',
            position: 'right',
            value: '<select id="language-switcher" style="padding: 0.25rem 0.5rem; border-radius: 4px; border: 1px solid var(--ifm-color-emphasis-300); background-color: var(--ifm-background-surface-color); color: var(--ifm-font-color-base); margin: 0 1rem;">' +
              '<option value="en">English</option>' +
              '<option value="ur">Urdu</option>' +
              '<option value="sd">Sindhi</option>' +
              '</select>',
            className: 'navbar__item',
          },
          {
            href: 'https://github.com/Anas-Rajput12/my-physicalAi-2',
            label: 'GitHub',
            position: 'right',
            className: 'navbar-github-link',
            'aria-label': 'GitHub profile',
          },
          {
            type: 'dropdown',
            position: 'right',
            label: 'Account',
            items: [
              {
                label: 'Sign In',
                to: '/my-physical-ai-book/signin',
              },
              {
                label: 'Sign Up',
                to: '/my-physical-ai-book/signup',
              }
            ]
          },
        ],
      },
      // Add custom fields to theme config
      clerkPublishableKey: process.env.REACT_APP_CLERK_PUBLISHABLE_KEY || '',
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Course',
            items: [
              {
                label: 'Book',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/Anas-Rajput12/my-physicalAi-2',
              },
            ],
          },
          {
            title: 'Legal',
            items: [
              {
                label: `© ${new Date().getFullYear()} Physical AI`,
                to: '/',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
      // Enable detailed bundle analytics for performance optimization
      // algolia: {
      //   // The application ID provided by Algolia
      //   appId: 'YOUR_APP_ID',
      //   // Public API key: it is safe to commit it
      //   apiKey: 'YOUR_SEARCH_API_KEY',
      //   indexName: 'YOUR_INDEX_NAME',
      // },
    }),

};

export default config;