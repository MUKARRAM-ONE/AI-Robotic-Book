module.exports = function (context, options) {
  return {
    name: 'my-proxy-plugin',
    configureWebpack(config, isServer) {
      if (!isServer) {
        return {
          devServer: {
            proxy: {
              '/api': {
                target: 'https://<your-ngrok-url>',
                pathRewrite: { '^/api': '' },
                secure: false,
                changeOrigin: true,
              },
            },
          },
        };
      }
      return {};
    },
  };
};
