const fs = require("fs");
const http = require("http");
const path = require("path");

const root = path.resolve(__dirname, "..");
const port = Number(process.env.PORT || 8787);

const mimeTypes = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".uf2": "application/octet-stream"
};

function send(res, status, body, type = "text/plain; charset=utf-8") {
  res.writeHead(status, {
    "Content-Type": type,
    "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
    "Pragma": "no-cache"
  });
  res.end(body);
}

const server = http.createServer((req, res) => {
  const url = new URL(req.url, `http://127.0.0.1:${port}`);
  const route = url.pathname === "/" ? "/tools/ns2-webhid-tuner.html" : url.pathname;
  const file = path.resolve(root, route.replace(/^\/+/, ""));

  if (!file.startsWith(root)) {
    send(res, 403, "Forbidden");
    return;
  }

  fs.readFile(file, (err, data) => {
    if (err) {
      send(res, 404, "Not found");
      return;
    }
    const type = mimeTypes[path.extname(file).toLowerCase()] || "application/octet-stream";
    send(res, 200, data, type);
  });
});

server.listen(port, "127.0.0.1", () => {
  console.log(`NS2Pro WebHID tuner: http://127.0.0.1:${port}/tools/ns2-webhid-tuner.html`);
});
