const express = require('express');
const bodyParser = require('body-parser');
const mysql = require('mysql2/promise');
const Docker = require('dockerode');
const fs = require('fs');
const path = require('path');

const app = express();
app.use(bodyParser.json());
app.use(express.static(__dirname));

// --- DB config ---
const {
  DB_HOST = 'db',
  DB_USER = 'rosapp',
  DB_PASS = 'Dpt-311011',
  DB_NAME = 'ros_logs',
  ROBOT_CONTAINER = 'tbt3_robot',
  PORT = 3000
} = process.env;

let pool;
(async () => {
  pool = await mysql.createPool({
    host: DB_HOST,
    user: DB_USER,
    password: DB_PASS,
    database: DB_NAME,
    waitForConnections: true,
    connectionLimit: 5
  });
})();

// --- Docker ---
const docker = new Docker({ socketPath: '/var/run/docker.sock' });

async function getContainer() {
  try {
    return docker.getContainer(ROBOT_CONTAINER);
  } catch {
    return null;
  }
}

// ===== Logs API =====
app.post('/api/log', async (req, res) => {
  try {
    const { event, detail='', level='INFO' } = req.body || {};
    await pool.execute(
      'INSERT INTO logs(action, state, status, details) VALUES (?,?,?,?)',
      [event || 'unknown', level, 'completed', detail]
    );
    res.json({ ok: true });
  } catch (e) {
    res.status(500).json({ ok:false, error: e.message });
  }
});

app.get('/api/logs', async (req, res) => {
  const minutes = Math.max(1, Math.min(360, parseInt(req.query.minutes || '60', 10)));
  const [rows] = await pool.query(
    'SELECT id, ts, action as event, state as level, status, details as detail FROM logs WHERE ts >= NOW() - INTERVAL ? MINUTE ORDER BY id DESC',
    [minutes]
  );
  res.json(rows);
});

// --- Robot power ---
app.post('/api/toggle', async (req, res) => {
  const action = String(req.query.action || '').toLowerCase();
  try {
    const c = await getContainer();
    if (!c) throw new Error('Robot container not found');

    if (action === 'start') {
      await c.start();
      await pool.execute('INSERT INTO logs(action, state, status, details) VALUES (?,?,?,?)', 
        ['robot_start', 'INFO', 'completed', 'Container started']);
      return res.json({ status: 'starting' });
    }
    if (action === 'stop') {
      await c.stop({ t: 2 });
      await pool.execute('INSERT INTO logs(action, state, status, details) VALUES (?,?,?,?)', 
        ['robot_stop', 'INFO', 'completed', 'Container stopped']);
      return res.json({ status: 'stopping' });
    }
    res.status(400).json({ status: 'error', error: 'Invalid action' });
  } catch (e) {
    await pool.execute('INSERT INTO logs(action, state, status, details) VALUES (?,?,?,?)', 
      ['robot_' + action, 'ERROR', 'failed', e.message]).catch(() => {});
    res.status(500).json({ status: 'error', error: e.toString() });
  }
});

app.get('/api/robot/health', async (req, res) => {
  try {
    const c = await getContainer();
    if (!c) return res.json({ running: false });
    const info = await c.inspect();
    res.json({ running: info.State.Running });
  } catch {
    res.json({ running: false });
  }
});

// === Snapshot endpoint ===
app.post('/api/snapshot', async (req, res) => {
  try {
    const { dataURL, topic='/camera/image_raw' } = req.body || {};
    if (!dataURL || !dataURL.startsWith('data:image/')) {
      return res.status(400).json({ok:false, error:'bad dataURL'});
    }
    const b64 = dataURL.split(',')[1];
    const buf = Buffer.from(b64, 'base64');
    const dir = path.join(__dirname, 'public', 'snaps');
    if (!fs.existsSync(dir)) fs.mkdirSync(dir, { recursive: true });
    const fname = `snap_${Date.now()}.jpg`;
    fs.writeFileSync(path.join(dir, fname), buf);
    await pool.execute('INSERT INTO logs(action, state, status, details) VALUES (?,?,?,?)',
      ['snapshot', 'INFO', 'completed', `topic=${topic} file=/snaps/${fname}`]);
    res.json({ ok:true, file:`/snaps/${fname}` });
  } catch (e) {
    res.status(500).json({ ok:false, error:e.message });
  }
});

// --- Start server ---
app.listen(PORT, () => console.log(`Web/API on ${PORT}`));
