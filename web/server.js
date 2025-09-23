const express = require('express');
const bodyParser = require('body-parser');
const mysql = require('mysql2/promise');
const Docker = require('dockerode');

const app = express();
app.use(bodyParser.json());
app.use(express.static('public'));

const {
  DB_HOST='127.0.0.1',
  DB_USER='rosapp',
  DB_PASS='Dpt-311011',
  DB_NAME='ros_logs'
} = process.env;

let pool;
(async () => {
  pool = await mysql.createPool({
    host: DB_HOST, user: DB_USER, password: DB_PASS, database: DB_NAME,
    waitForConnections: true, connectionLimit: 5
  });
})();

app.post('/api/log', async (req, res) => {
  try {
    const { action, state, status, details } = req.body || {};
    await pool.execute(
      'INSERT INTO logs(action,state,status,details) VALUES(?,?,?,?)',
      [action||null, state||null, status||null, details||null]
    );
    res.json({ok:true});
  } catch (e) {
    res.status(500).json({ok:false, error: e.toString()});
  }
});

app.get('/api/log/last-hour', async (req, res) => {
  const [rows] = await pool.query(
    'SELECT * FROM logs WHERE ts >= NOW() - INTERVAL 1 HOUR ORDER BY ts DESC'
  );
  res.json(rows);
});

/** POWER: start/stop the robot container (local dev only) */
const docker = new Docker({ socketPath: '/var/run/docker.sock' });
const ROBOT_NAME = 'tbt3_robot';

async function ensureContainer(name) {
  try { return docker.getContainer(name); } catch { return null; }
}

app.post('/api/power/start', async (req, res) => {
  try {
    const c = await ensureContainer(ROBOT_NAME);
    await c.start();
    res.json({status:'starting'});
  } catch (e) { res.status(500).json({status:'error', error:e.toString()}); }
});

app.post('/api/power/stop', async (req, res) => {
  try {
    const c = await ensureContainer(ROBOT_NAME);
    await c.stop({t: 2});
    res.json({status:'stopping'});
  } catch (e) { res.status(500).json({status:'error', error:e.toString()}); }
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, ()=> console.log(`Web/API on ${PORT}`));
