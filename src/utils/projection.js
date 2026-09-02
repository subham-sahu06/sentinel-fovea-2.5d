/**
 * 2.5D perspective viewport projection and scaling utilities.
 */

export function projectWorldPoint(worldX, worldY, worldZ, pose, width, height) {
  const relativeX = worldX - pose.x;
  const relativeY = worldY - pose.y;
  const rotatedX = relativeX * Math.cos(-pose.yaw) - relativeY * Math.sin(-pose.yaw);
  const rotatedY = relativeX * Math.sin(-pose.yaw) + relativeY * Math.cos(-pose.yaw);
  const depth = Math.max(0.2, 1 + rotatedY / 12);
  return {
    x: width / 2 + rotatedX * 31 * depth,
    y: height * 0.58 - worldZ * 42 * depth + rotatedY * 13 * depth,
    depth,
    size: Math.max(2, 31 * resolutionScale(depth)),
  };
}

export function resolutionScale(depth) {
  return Math.min(1.8, Math.max(0.45, depth));
}

