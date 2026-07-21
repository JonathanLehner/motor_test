const canvas = document.getElementById("force-map");
const ctx = canvas.getContext("2d");
const tooltip = document.getElementById("tooltip");
let latestPoints = [];
let screenPoints = [];

const $ = (id) => document.getElementById(id);
const fixed = (value, digits = 1) => Number(value).toFixed(digits);

function colorForForce(value) {
  const t = Math.max(0, Math.min(1, value / 25));
  const stops = [
    [0.00, [24, 70, 73]],
    [0.28, [22, 196, 194]],
    [0.68, [255, 224, 99]],
    [1.00, [255, 87, 58]],
  ];
  let a = stops[0], b = stops[stops.length - 1];
  for (let i = 1; i < stops.length; i++) {
    if (t <= stops[i][0]) { a = stops[i - 1]; b = stops[i]; break; }
  }
  const u = (t - a[0]) / Math.max(b[0] - a[0], 0.001);
  const rgb = a[1].map((v, i) => Math.round(v + (b[1][i] - v) * u));
  return `rgb(${rgb.join(",")})`;
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * ratio);
  canvas.height = Math.round(rect.height * ratio);
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  drawMap(latestPoints);
}

function drawArrow(x, y, dx, dy, color) {
  const length = Math.hypot(dx, dy);
  if (length < 2) return;
  const angle = Math.atan2(dy, dx);
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + dx, y + dy); ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(x + dx, y + dy);
  ctx.lineTo(x + dx - 6 * Math.cos(angle - 0.5), y + dy - 6 * Math.sin(angle - 0.5));
  ctx.lineTo(x + dx - 6 * Math.cos(angle + 0.5), y + dy - 6 * Math.sin(angle + 0.5));
  ctx.closePath(); ctx.fill();
}

function drawMap(points) {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  ctx.clearRect(0, 0, width, height);
  if (!points || !points.length) {
    ctx.fillStyle = "#789b91";
    ctx.font = "14px system-ui";
    ctx.textAlign = "center";
    ctx.fillText("Waiting for sensor data…", width / 2, height / 2);
    return;
  }

  const xs = points.map(p => p.x), ys = points.map(p => p.y);
  const minX = Math.min(...xs), maxX = Math.max(...xs);
  const minY = Math.min(...ys), maxY = Math.max(...ys);
  const padX = Math.max(68, width * 0.16), padY = 52;
  const scale = Math.min((width - 2 * padX) / (maxX - minX), (height - 2 * padY) / (maxY - minY));
  const usedW = (maxX - minX) * scale, usedH = (maxY - minY) * scale;
  const originX = (width - usedW) / 2 - minX * scale;
  const originY = (height + usedH) / 2 + minY * scale;
  const project = p => ({ x: originX + p.x * scale, y: originY - p.y * scale });

  ctx.save();
  ctx.strokeStyle = "rgba(111,255,209,.14)";
  ctx.fillStyle = "rgba(5,26,22,.66)";
  ctx.lineWidth = 1.5;
  const outlineX = originX + (minX - 1.4) * scale;
  const outlineY = originY - (maxY + 1.4) * scale;
  const outlineW = (maxX - minX + 2.8) * scale;
  const outlineH = (maxY - minY + 2.8) * scale;
  ctx.beginPath();
  ctx.roundRect(outlineX, outlineY, outlineW, outlineH, Math.min(outlineW / 2, 80));
  ctx.fill(); ctx.stroke();

  screenPoints = points.map(point => ({ ...project(point), point }));
  for (const item of screenPoints) {
    const p = item.point;
    const radius = 7 + Math.min(p.fz, 25) * 0.48;
    const color = colorForForce(p.fz);
    ctx.shadowColor = color;
    ctx.shadowBlur = p.fz > 0.2 ? 13 : 3;
    ctx.fillStyle = color;
    ctx.globalAlpha = 0.78 + Math.min(p.fz / 25, 1) * 0.22;
    ctx.beginPath(); ctx.arc(item.x, item.y, radius, 0, Math.PI * 2); ctx.fill();
    ctx.shadowBlur = 0; ctx.globalAlpha = 1;
    ctx.fillStyle = p.fz > 10 ? "#142018" : "#dffff4";
    ctx.font = "700 9px ui-monospace, monospace";
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText(String(p.index), item.x, item.y + 0.5);
    drawArrow(item.x, item.y, p.fx * 7, -p.fy * 7, "rgba(232,255,247,.88)");
  }
  ctx.restore();
}

function updateStats(data) {
  const status = $("status");
  if (data.error) {
    status.className = "status error";
    status.querySelector("b").textContent = data.error;
  } else if (data.frame) {
    status.className = "status live";
    status.querySelector("b").textContent = "Live";
  } else {
    status.className = "status waiting";
    status.querySelector("b").textContent = "Connecting";
  }
  $("port").textContent = data.port || "—";
  $("address").textContent = data.device_address ?? "—";
  $("rate").textContent = data.sample_count ? `${fixed(data.average_hz, 1)} Hz` : "—";
  $("count").textContent = data.sample_count ?? 0;

  if (!data.frame) return;
  const total = data.frame.resultant;
  $("total-fx").textContent = fixed(total.fx);
  $("total-fy").textContent = fixed(total.fy);
  $("total-fz").textContent = fixed(total.fz);
  $("total-mag").textContent = `${fixed(total.magnitude)} N`;
  latestPoints = data.frame.points;
  drawMap(latestPoints);

  const hottest = [...latestPoints].sort((a, b) => b.fz - a.fz).slice(0, 5);
  $("ranking").innerHTML = hottest.map(point => `
    <li><b>${point.index}</b><span><i style="width:${Math.min(point.fz / 25 * 100, 100)}%"></i></span><em>${fixed(point.fz)} N</em></li>
  `).join("");
}

async function poll() {
  try {
    const response = await fetch("/api/frame", { cache: "no-store" });
    updateStats(await response.json());
  } catch (error) {
    updateStats({ error: `UI connection failed: ${error.message}` });
  } finally {
    // Sensor polling is 83.3 Hz. Browsers normally render at the display
    // refresh rate (commonly 60 Hz), so fetch the latest sample once per frame.
    window.setTimeout(poll, 16);
  }
}

canvas.addEventListener("mousemove", event => {
  const rect = canvas.getBoundingClientRect();
  const x = event.clientX - rect.left, y = event.clientY - rect.top;
  let nearest = null, distance = Infinity;
  for (const item of screenPoints) {
    const d = Math.hypot(item.x - x, item.y - y);
    if (d < distance) { distance = d; nearest = item; }
  }
  if (!nearest || distance > 25) { tooltip.style.display = "none"; return; }
  const p = nearest.point;
  tooltip.innerHTML = `<b>P${String(p.index).padStart(2,"0")}</b><br>Fx ${fixed(p.fx)} N · Fy ${fixed(p.fy)} N<br>Fz ${fixed(p.fz)} N<br><small>(${fixed(p.x,2)}, ${fixed(p.y,2)}, ${fixed(p.z,2)}) mm</small>`;
  tooltip.style.display = "block";
  tooltip.style.left = `${Math.min(x + 14, rect.width - 165)}px`;
  tooltip.style.top = `${Math.max(y - 70, 4)}px`;
});
canvas.addEventListener("mouseleave", () => { tooltip.style.display = "none"; });
window.addEventListener("resize", resizeCanvas);
resizeCanvas();
poll();
