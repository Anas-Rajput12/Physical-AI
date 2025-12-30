const path = require('path');

module.exports = function (context, options) {
  return {
    name: 'docusaurus-plugin-rag-chatbot',

    getClientModules() {
      return [path.resolve(__dirname, './rag-chatbot-client.js')];
    },

    configureWebpack(config, isServer, utils) {
      return {
        resolve: {
          alias: {
            '@chatbot-config': path.resolve(__dirname, '../../../config/chatbot.config.js'),
          },
        },
      };
    },
  };
};