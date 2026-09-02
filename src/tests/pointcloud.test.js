import test from 'node:test';
import assert from 'node:assert/strict';
import { parsePointCloud, SEMANTIC_CLASSES } from '../utils/pointcloud.js';

test('SEMANTIC_CLASSES contains 4 required DRDO classes', () => {
  assert.equal(SEMANTIC_CLASSES[0].name, 'DRIVABLE');
  assert.equal(SEMANTIC_CLASSES[1].name, 'NEGATIVE / TRENCH');
  assert.equal(SEMANTIC_CLASSES[2].name, 'STATIC OBSTACLE');
  assert.equal(SEMANTIC_CLASSES[3].name, 'DYNAMIC TARGET');
});

test('parsePointCloud parses binary Uint8Array points correctly with semantic classes', () => {
  const points = [
    { x: 1.5, y: -2.0, z: 0.02, intensity: 30.0, class_id: 0.0, confidence: 0.95 },   // Drivable
    { x: 6.0, y: 0.0, z: -0.40, intensity: 10.0, class_id: 1.0, confidence: 0.92 },   // Trench / Negative
    { x: 14.0, y: -4.5, z: 0.80, intensity: 90.0, class_id: 2.0, confidence: 0.88 },  // Static Wall
    { x: 8.5, y: 0.5, z: 1.20, intensity: 200.0, class_id: 3.0, confidence: 0.96 },   // Dynamic Target
  ];

  const buffer = new ArrayBuffer(points.length * 24);
  const view = new DataView(buffer);
  points.forEach((p, idx) => {
    view.setFloat32(idx * 24 + 0, p.x, true);
    view.setFloat32(idx * 24 + 4, p.y, true);
    view.setFloat32(idx * 24 + 8, p.z, true);
    view.setFloat32(idx * 24 + 12, p.intensity, true);
    view.setFloat32(idx * 24 + 16, p.class_id, true);
    view.setFloat32(idx * 24 + 20, p.confidence, true);
  });

  const message = {
    fields: [
      { name: 'x', offset: 0 },
      { name: 'y', offset: 4 },
      { name: 'z', offset: 8 },
      { name: 'intensity', offset: 12 },
      { name: 'class_id', offset: 16 },
      { name: 'confidence', offset: 20 },
    ],
    width: 4,
    height: 1,
    point_step: 24,
    is_bigendian: false,
    data: new Uint8Array(buffer),
  };

  const parsed = parsePointCloud(message);
  assert.equal(parsed.length, 4);

  assert.equal(parsed[0].class_id, 0);
  assert.equal(parsed[0].obstacle, false);

  assert.equal(parsed[1].class_id, 1);
  assert.equal(parsed[1].obstacle, true);

  assert.equal(parsed[2].class_id, 2);
  assert.equal(parsed[2].obstacle, true);

  assert.equal(parsed[3].class_id, 3);
  assert.equal(parsed[3].obstacle, true);
});

test('parsePointCloud handles Base64 encoded data string', () => {
  const buffer = new ArrayBuffer(12);
  const view = new DataView(buffer);
  view.setFloat32(0, 2.5, true);
  view.setFloat32(4, 3.5, true);
  view.setFloat32(8, 0.85, true);

  const uint8 = new Uint8Array(buffer);
  const base64 = btoa(String.fromCharCode(...uint8));

  const message = {
    fields: [
      { name: 'x', offset: 0 },
      { name: 'y', offset: 4 },
      { name: 'z', offset: 8 },
    ],
    width: 1,
    height: 1,
    point_step: 12,
    is_bigendian: false,
    data: base64,
  };

  const parsed = parsePointCloud(message);
  assert.equal(parsed.length, 1);
  assert.equal(Math.abs(parsed[0].x - 2.5) < 1e-4, true);
  assert.equal(Math.abs(parsed[0].y - 3.5) < 1e-4, true);
  assert.equal(parsed[0].obstacle, true);
});

test('parsePointCloud safely handles empty or malformed inputs', () => {
  assert.deepEqual(parsePointCloud({}), []);
  assert.deepEqual(parsePointCloud({ data: null, fields: [] }), []);
  assert.deepEqual(parsePointCloud({ data: new Uint8Array([]), fields: [] }), []);
});
