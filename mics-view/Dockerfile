# MICS-View frontend. Built and served entirely in Docker — no host npm install.
#
#   build stage : node, runs `npm ci` + `vite build` (vite-plugin-cesium copies
#                 the self-hosted Cesium assets into dist/, so the bundle is
#                 air-gappable — no Cesium Ion at runtime).
#   prod stage  : nginx serving the static dist/ with SPA fallback.
#   dev stage   : node running the Vite dev server (HMR) for local iteration.
#
# The browser runs OUTSIDE Docker and connects to the gateway at ws://<host>:8080;
# the UI derives that URL from the page host unless VITE_WS_URL is baked in.

# ---- build -----------------------------------------------------------------
FROM node:20-bookworm-slim AS build
WORKDIR /app
COPY package.json package-lock.json* ./
# package-lock may be absent on first build; fall back to a plain install.
RUN if [ -f package-lock.json ]; then npm ci; else npm install; fi
COPY tsconfig*.json vite.config.ts index.html ./
COPY src ./src
RUN npm run build

# ---- prod (default) --------------------------------------------------------
FROM nginx:1.27-alpine AS prod
COPY nginx.conf /etc/nginx/conf.d/default.conf
COPY --from=build /app/dist /usr/share/nginx/html
EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]

# ---- dev -------------------------------------------------------------------
FROM node:20-bookworm-slim AS dev
WORKDIR /app
COPY package.json package-lock.json* ./
RUN if [ -f package-lock.json ]; then npm ci; else npm install; fi
COPY . .
EXPOSE 5173
CMD ["npm", "run", "dev"]
