// shoot.mjs — headless screenshots of the visualizer for sample pairs.
//   NODE_PATH=$(npm root -g) node viz/shoot.mjs [pair ...]
// Serves the viz/ dir over http (ES modules + fetch need a real origin),
// then drives chromium to each ?pair=N and writes viz/shots/pair-N.png.
import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);
// ESM ignores NODE_PATH; resolve the globally-installed playwright explicitly.
const { chromium } = require(process.env.PLAYWRIGHT_PKG || 'playwright');
import http from 'node:http';
import { readFile } from 'node:fs/promises';
import { mkdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const DIR = path.dirname(fileURLToPath(import.meta.url));
const MIME = { '.html': 'text/html', '.js': 'text/javascript',
               '.txt': 'text/plain', '.png': 'image/png' };

const server = http.createServer(async (req, res) => {
  try {
    const rel = decodeURIComponent(req.url.split('?')[0]).replace(/^\/+/, '') || 'index.html';
    const buf = await readFile(path.join(DIR, rel));
    res.writeHead(200, { 'Content-Type': MIME[path.extname(rel)] || 'application/octet-stream' });
    res.end(buf);
  } catch { res.writeHead(404); res.end('not found'); }
});

const pairs = process.argv.slice(2).map(Number);
const PAIRS = pairs.length ? pairs : [0, 1, 3, 4, 6, 7, 117, 805];

await new Promise(r => server.listen(0, r));
const port = server.address().port;
mkdirSync(path.join(DIR, 'shots'), { recursive: true });

const browser = await chromium.launch();
const page = await browser.newPage({ viewport: { width: 1280, height: 860 }, deviceScaleFactor: 2 });
page.on('pageerror', e => console.error('PAGE ERROR:', e.message));
page.on('console', m => { if (m.type() === 'error') console.error('console:', m.text()); });

for (const n of PAIRS) {
  await page.goto(`http://localhost:${port}/index.html?pair=${n}`, { waitUntil: 'load' });
  await page.waitForFunction(() => window.__ready === true, { timeout: 15000 });
  await page.waitForTimeout(700); // let OrbitControls damping settle + render
  const info = await page.$eval('#info', el => el.innerText.replace(/\s+/g, ' ').trim());
  const out = path.join(DIR, 'shots', `pair-${String(n).padStart(4, '0')}.png`);
  await page.screenshot({ path: out });
  console.log(`pair ${n}: ${out}\n   ${info}`);
}

await browser.close();
server.close();
