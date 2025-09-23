FROM node:18-alpine

WORKDIR /app

# Copy package.json first for better caching
COPY package.json ./
RUN npm ci --omit=dev || npm install --omit=dev

# Create public directory
RUN mkdir -p public

# Copy application files
COPY server.js ./
COPY index.html ./public/

EXPOSE 3000
CMD [ "node", "server.js" ]
