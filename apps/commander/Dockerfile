FROM node:22-alpine
WORKDIR /app

COPY package.json package-lock.json ./
RUN npm ci

COPY tsconfig.json tsconfig.build.json nest-cli.json ./
COPY src ./src

RUN npm run build

ENV NODE_ENV=production
EXPOSE 3001

CMD ["node", "dist/main.js"]
