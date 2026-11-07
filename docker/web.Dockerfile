# syntax=docker/dockerfile:1
FROM node:24-slim

WORKDIR /app
COPY --from=web package.json package-lock.json ./
RUN --mount=type=cache,target=/root/.npm npm ci
COPY --from=web --exclude=node_modules --exclude=dist --exclude=.git . .

EXPOSE 5173
CMD ["npm", "run", "dev", "--", "--host", "0.0.0.0", "--port", "5173", "--strictPort"]
