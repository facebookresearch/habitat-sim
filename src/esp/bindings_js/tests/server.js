import http from "http";
import finalhandler from "finalhandler";
import serveStatic from "serve-static";

export async function createServer() {
  const serve = serveStatic("./");
  const server = http.createServer(function(req, res) {
    var done = finalhandler(req, res);
    serve(req, res, done);
  });

  await new Promise((resolve, reject) => {
    const startServer = () => {
      server.once("error", e => {
        if (e.code === "EADDRINUSE") {
          server.close(startServer);
        }
      });
      server.listen(4004, "localhost", err => {
        if (err) {
          reject("Failed to listen on port 4004");
        } else {
          const address = server.address();
          console.log(`Listening on http://${address.address}:${address.port}`);
          resolve();
        }
      });
    };
    startServer();
  });
  return server;
}

createServer();
