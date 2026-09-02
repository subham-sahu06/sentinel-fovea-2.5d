/**
 * Point cloud parsing and semantic classification utilities.
 * Supports standard PointCloud2 and Deep Learning Semantic PointCloud2 formats.
 */

export const SEMANTIC_CLASSES = {
  0: { name: 'DRIVABLE', color: 'rgba(52, 211, 153, 0.85)', hex: '#34d399', rgb: [52, 211, 153] },
  1: { name: 'NEGATIVE / TRENCH', color: 'rgba(251, 146, 60, 0.95)', hex: '#fb923c', rgb: [251, 146, 60] },
  2: { name: 'STATIC OBSTACLE', color: 'rgba(248, 113, 113, 0.90)', hex: '#f87171', rgb: [248, 113, 113] },
  3: { name: 'DYNAMIC TARGET', color: 'rgba(56, 189, 248, 0.95)', hex: '#38bdf8', rgb: [56, 189, 248] },
};

export function parsePointCloud(message) {
  if (!message || !message.data || !message.fields) return [];
  const fields = Object.fromEntries(message.fields.map((field) => [field.name, field]));
  const bytes = typeof message.data === 'string'
    ? Uint8Array.from(atob(message.data), (character) => character.charCodeAt(0))
    : new Uint8Array(message.data);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const points = [];
  const stride = message.point_step || 0;
  if (stride <= 0) return [];

  const rawCount = message.width * (message.height || 1);
  // Subsample if very dense to keep rendering at a smooth 60 FPS
  const step = rawCount > 5000 ? Math.ceil(rawCount / 4000) : 1;
  const count = Math.min(rawCount, 25000);

  const hasClass = 'class_id' in fields;
  const hasConf = 'confidence' in fields;
  const hasIntensity = 'intensity' in fields;

  for (let index = 0; index < count; index += step) {
    const offset = index * stride;
    if (offset + stride > bytes.length) break;

    const get = (name) => {
      const field = fields[name];
      return field ? view.getFloat32(offset + field.offset, !message.is_bigendian) : NaN;
    };

    const x = get('x');
    const y = get('y');
    const z = get('z');

    if (Number.isFinite(x) && Number.isFinite(y) && Number.isFinite(z)) {
      const classId = hasClass ? Math.round(get('class_id')) : (z < -0.10 ? 1 : z > 0.15 ? 2 : 0);
      const conf = hasConf ? get('confidence') : 0.92;
      const intensity = hasIntensity ? get('intensity') : 50;

      points.push({
        x,
        y,
        z,
        class_id: classId,
        confidence: Number.isFinite(conf) ? conf : 0.90,
        intensity: Number.isFinite(intensity) ? intensity : 50,
        obstacle: classId !== 0,
      });
    }
  }
  return points;
}
