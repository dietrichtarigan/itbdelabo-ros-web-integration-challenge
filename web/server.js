const express = require('express');
const path = require('path');
const morgan = require('morgan');
const cors = require('cors');
const axios = require('axios');
const mysql = require('mysql2/promise');

const app = express();
app.use(express.json());
app.use(cors());
app.use(morgan('dev'));

const {
  DB_HOST = 'db',
  DB_NAME = 'rosweb',
  DB_USER = 'ros',
  DB_PASSWORD = 'ros',
  ROBOT_API = 'http://robot:5000'
} = process.env;

// MySQL pool
let pool;
(async () => {
  pool = await mysql.createPool({
    host: DB_HOST,
    user: DB_USER,
    password: DB_PASSWORD,
    database: DB_NAME,
    waitForConnections: true,
    connectionLimit: 10
  });
})();

// Helper: write a log
async function log(event, detail = '', level = 'INFO') {
  try {
    await pool.query('INSERT INTO logs (event, detail, level) VALUES (?, ?, ?)', [event, detail, level]);
  } catch (e) {
    console.error('log insert failed:', e.message);
  }
}

// Serve the SPA
app.get('/', (_, res) => res.sendFile(path.join(__dirname, 'index.html')));

// Toggle robot simulation (start/stop)
app.post('/api/toggle', async (req, res) => {
  try {
    const { action } = req.query; // ?action=start or stop
    if (!action || !['start', 'stop'].includes(action)) {
      return res.status(400).json({ error: "Query param 'action' must be 'start' or 'stop'." });
    }
    const url = `${ROBOT_API}/${action}`;
    const { data } = await axios.post(url).catch(async (err) => {
      // Retry via GET in case POST blocked by proxy
      if (err.response) throw err;
      const r = await axios.get(url);
      return { data: r.data };
    });
    await log('toggle', `action=${action} status=${data.status}`);
    res.json(data);
  } catch (err) {
    await log('toggle_error', String(err), 'ERROR');
    res.status(500).json({ error: err.message });
  }
});

// Endpoint to record any client-side actions (e.g., move button presses)
app.post('/api/log', async (req, res) => {
  const { event, detail, level } = req.body || {};
  if (!event) return res.status(400).json({ error: 'event is required' });
  await log(event, detail || '', level || 'INFO');
  res.json({ ok: true });
});

// Fetch logs from the last N minutes (default 60)
app.get('/api/logs', async (req, res) => {
  const minutes = Math.max(1, Math.min(240, Number(req.query.minutes || 60)));
  const [rows] = await pool.query(
    `SELECT id, ts, level, event, detail
     FROM logs
     WHERE ts >= (NOW() - INTERVAL ? MINUTE)
     ORDER BY ts DESC`,
    [minutes]
  );
  res.json(rows);
});

// Health of the robot sim (running or not)
app.get('/api/robot/health', async (_req, res) => {
  try {
    const { data } = await axios.get(`${ROBOT_API}/health`);
    res.json(data);
  } catch (e) {
    res.status(200).json({ running: false, note: 'robot control unreachable' });
  }
});

// Static (if you later add assets)
app.use(express.static(__dirname));

const PORT = process.env.PORT || 8000;
app.listen(PORT, () => console.log(`Web server running on http://0.0.0.0:${PORT}`));
