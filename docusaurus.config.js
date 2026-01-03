// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';
import 'dotenv/config';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive course for senior undergraduate AI students',
  favicon: 'img/favicon.ico',

  /* =========================
     DEPLOYMENT (GITHUB PAGES / VERCEL)
     ========================= */
  url: 'https://hackathon-quarter4-alpha.vercel.app/',
  baseUrl: '/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  trailingSlash: false,

  /* =========================
     SCRIPTS & HEAD TAGS
     ========================= */
  scripts: [
    {
      src: 'js/language-switcher.js',
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
      attributes: {},
      innerHTML: `
        document.addEventListener('DOMContentLoaded', () => {
          const switcher = document.getElementById('language-switcher');
          if (!switcher) return;

          const savedLang = localStorage.getItem('preferredLanguage') || 'en';
          switcher.value = savedLang;

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
<<<<<<< HEAD
    // Expose environment variables to the client
    {
      tagName: 'script',
      attributes: {
        type: 'text/javascript'
      },
      innerHTML: `
        window.REACT_APP_OPENROUTER_API_KEY = "${process.env.OPENROUTER_API_KEY || ''}";
        window.REACT_APP_CLERK_PUBLISHABLE_KEY = "${process.env.NEXT_PUBLIC_CLERK_PUBLISHABLE_KEY || ''}";
=======
    {
      tagName: 'script',
      attributes: {}, // ✅ attributes must be present
      innerHTML: `
        window.REACT_APP_OPENROUTER_API_KEY = "${process.env.OPENROUTER_API_KEY || ''}";
>>>>>>> e4c82aa21aed9bd3cb31fd8112fd270a81d758cd
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
          editUrl: 'https://github.com/Anas-Rajput12/Hackathon-quarter4',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  /* =========================
     GITHUB PAGES DEPLOYMENT
     ========================= */
  organizationName: 'Anas-Rajput12',
  projectName: 'Hackathon-quarter4',
  deploymentBranch: 'gh-pages',

  /* =========================
     THEME CONFIG
     ========================= */
  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
<<<<<<< HEAD
    // Clerk publishable key - replace with your actual key
=======
>>>>>>> e4c82aa21aed9bd3cb31fd8112fd270a81d758cd
    clerkPublishableKey: process.env.NEXT_PUBLIC_CLERK_PUBLISHABLE_KEY || '',

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
<<<<<<< HEAD

        // {
        //   type: 'html',
        //   position: 'right',
        //   value: `
        //     <select id="language-switcher"
        //       style="
        //         padding: 0.3rem 0.6rem;
        //         border-radius: 6px;
        //         border: 1px solid var(--ifm-color-emphasis-300);
        //         background-color: var(--ifm-background-surface-color);
        //         color: var(--ifm-font-color-base);
        //       ">
        //       <option value="en">English</option>
        //       <option value="ur">Urdu</option>
        //       <option value="sd">Sindhi</option>
        //     </select>
        //   `,
        // },

=======
        {
          type: 'html',
          position: 'right',
          value: `
            <select id="language-switcher"
              style="
                padding: 0.3rem 0.6rem;
                border-radius: 6px;
                border: 1px solid var(--ifm-color-emphasis-300);
                background-color: var(--ifm-background-surface-color);
                color: var(--ifm-font-color-base);
              ">
              <option value="en">English</option>
              <option value="ur">Urdu</option>
              <option value="sd">Sindhi</option>
            </select>
          `,
        },
>>>>>>> e4c82aa21aed9bd3cb31fd8112fd270a81d758cd
        {
          href: 'https://github.com/Anas-Rajput12/Hackathon-quarter4',
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
              href: 'https://github.com/Anas-Rajput12/Hackathon-quarter4',
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
