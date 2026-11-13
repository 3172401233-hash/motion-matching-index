export function wrapAngle(angle) {
  let wrapped = angle % (2 * Math.PI);
  if (wrapped > Math.PI) wrapped -= 2 * Math.PI;
  if (wrapped <= -Math.PI) wrapped += 2 * Math.PI;
  return wrapped;
}

export function angleDistance(a, b) {
  return wrapAngle(a - b);
}

export function toLocal(x, y, heading) {
  const cosine = Math.cos(-heading);
  const sine = Math.sin(-heading);
  return {
    x: x * cosine - y * sine,
    y: x * sine + y * cosine,
  };
}

export function squaredDistance(ax, ay, bx, by) {
  const dx = ax - bx;
  const dy = ay - by;
  return dx * dx + dy * dy;
}

export function isFiniteNumber(value) {
  return typeof value === 'number' && Number.isFinite(value);
}

export function assertFiniteNumber(value, name) {
  if (!isFiniteNumber(value)) {
    throw new TypeError(`${name} must be a finite number, received ${String(value)}`);
  }
}
