FROM node:18-alpine

WORKDIR /app

COPY package.json ./
RUN npm install --only=production

COPY server.js index.html ./

EXPOSE 8000
CMD ["node", "server.js"]
