#!/usr/bin/env python3

import http.server
import socketserver
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--port", "-p", help="port to bind to",
                    type=int, required=False, default=8000)
args = parser.parse_args();

Handler = http.server.SimpleHTTPRequestHandler
Handler.extensions_map.update({
    '.wasm': 'application/wasm',
})

socketserver.TCPServer.allow_reuse_address = True
with socketserver.TCPServer(("", args.port), Handler) as httpd:
    httpd.allow_reuse_address = True
    print("serving at port", args.port)
    httpd.serve_forever()

