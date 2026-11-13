import { DEFAULT_HORIZONS, TRAJECTORY_TOLERANCE } from './features.js';
import { isFiniteNumber } from './math.js';

export const DEFAULT_WEIGHTS = Object.freeze({
  pose: 1,
  motion: 1,
  trajectory: 1,
});

const POSE_DIMENSIONS = [
  'leftFoot.x',
  'leftFoot.y',
  'rightFoot.x',
  'rightFoot.y',
];
const MOTION_DIMENSIONS = [
  'velocity.x',
  'velocity.y',
  'acceleration.x',
  'acceleration.y',
];

function trajectoryDimensions(horizons) {
  return horizons.flatMap((horizon) => [
    `trajectory@${horizon}s.x`,
    `trajectory@${horizon}s.y`,
    `trajectory@${horizon}s.heading`,
  ]);
}

export function defaultScales(horizons = DEFAULT_HORIZONS) {
  const scales = {};
  for (const dimension of [...POSE_DIMENSIONS, ...MOTION_DIMENSIONS, ...trajectoryDimensions(horizons)]) {
    scales[dimension] = 1;
  }
  return scales;
}

export function normalizeConfig(config = {}) {
  const horizons = config.horizons ?? DEFAULT_HORIZONS;
  if (!Array.isArray(horizons) || horizons.length === 0) {
    throw new TypeError('config.horizons must be a non-empty array');
  }
  horizons.forEach((horizon, index) => {
    if (!isFiniteNumber(horizon) || horizon <= 0) {
      throw new TypeError(`config.horizons[${index}] must be a positive finite number`);
    }
  });

  const tolerance = config.trajectoryTolerance ?? TRAJECTORY_TOLERANCE;
  if (!isFiniteNumber(tolerance) || tolerance < 0) {
    throw new TypeError('config.trajectoryTolerance must be a non-negative finite number');
  }

  const weights = { ...DEFAULT_WEIGHTS, ...(config.weights ?? {}) };
  for (const key of Object.keys(DEFAULT_WEIGHTS)) {
    if (!isFiniteNumber(weights[key]) || weights[key] < 0) {
      throw new TypeError(`config.weights.${key} must be a non-negative finite number`);
    }
  }

  const expectedDimensions = [
    ...POSE_DIMENSIONS,
    ...MOTION_DIMENSIONS,
    ...trajectoryDimensions(horizons),
  ];
  const scales = defaultScales(horizons);
  const customScales = config.scales ?? {};
  for (const dimension of expectedDimensions) {
    if (dimension in customScales) {
      const scale = customScales[dimension];
      if (!isFiniteNumber(scale) || scale <= 0) {
        throw new TypeError(`config.scales.${dimension} must be a positive finite number`);
      }
      scales[dimension] = scale;
    }
  }
  const unknown = Object.keys(customScales).filter((key) => !(key in scales));
  if (unknown.length > 0) {
    throw new TypeError(`Unknown scale dimension(s): ${unknown.join(', ')}`);
  }

  return {
    horizons: [...horizons],
    trajectoryTolerance: tolerance,
    weights,
    scales,
  };
}

export function normalizeQueryOptions(options = {}, config) {
  const topK = options.topK ?? 10;
  if (!Number.isInteger(topK) || topK <= 0) {
    throw new TypeError('options.topK must be a positive integer');
  }
  const excludedClipIds = options.excludedClipIds ?? [];
  if (!Array.isArray(excludedClipIds) || !excludedClipIds.every((id) => typeof id === 'string')) {
    throw new TypeError('options.excludedClipIds must be an array of clip id strings');
  }
  const maxCost = options.maxCost ?? null;
  if (maxCost !== null && (!isFiniteNumber(maxCost) || maxCost < 0)) {
    throw new TypeError('options.maxCost must be a non-negative finite number or null');
  }
  return {
    topK,
    excludedClipIds: new Set(excludedClipIds),
    maxCost,
    config,
  };
}
