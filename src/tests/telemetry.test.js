import test from 'node:test';
import assert from 'node:assert/strict';
import { projectWorldPoint, resolutionScale } from '../utils/projection.js';

test('resolutionScale bounds scale factor between 0.45 and 1.8', () => {
  assert.equal(resolutionScale(0.1), 0.45);
  assert.equal(resolutionScale(1.0), 1.0);
  assert.equal(resolutionScale(2.5), 1.8);
});

test('projectWorldPoint projects world coordinates with ego pose offset and rotation', () => {
  const width = 800;
  const height = 600;
  const pose = { x: 0, y: 0, yaw: 0 };

  // Origin point projection
  const origin = projectWorldPoint(0, 0, 0, pose, width, height);
  assert.equal(origin.x, width / 2);
  assert.equal(origin.y, height * 0.58);
  assert.equal(origin.depth, 1.0);

  // Point ahead (Y > 0 in robot local frame)
  const forwardPoint = projectWorldPoint(0, 12, 0, pose, width, height);
  assert.equal(forwardPoint.depth, 2.0); // 1 + 12/12 = 2.0
  assert.equal(forwardPoint.x, width / 2);

  // Point with robot rotated by 90 degrees (yaw = pi/2)
  const rotatedPose = { x: 0, y: 0, yaw: Math.PI / 2 };
  const pointX = projectWorldPoint(10, 0, 0, rotatedPose, width, height);
  assert.equal(Math.abs(pointX.y - (height * 0.58 - 10 * 13 * pointX.depth)) < 1e-3, true);
});
