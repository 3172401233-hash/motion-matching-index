import { normalizeConfig, normalizeQueryOptions } from './config.js';
import { extractFrameFeatures, validateFrame } from './features.js';

function buildClipFeatures(clip, horizons, tolerance) {
  if (clip === null || typeof clip !== 'object') {
    throw new TypeError('Each clip must be an object');
  }
  if (typeof clip.id !== 'string' || clip.id.length === 0) {
    throw new TypeError('Each clip requires a non-empty string id');
  }
  if (!Array.isArray(clip.frames) || clip.frames.length === 0) {
    throw new TypeError(`Clip "${clip.id}" must contain at least one frame`);
  }
  clip.frames.forEach((frame, index) => validateFrame(frame, `clip "${clip.id}" frame[${index}]`));
  clip.frames.forEach((frame, index) => {
    if (index > 0 && frame.time <= clip.frames[index - 1].time) {
      throw new TypeError(`Clip "${clip.id}" frame times must be strictly increasing`);
    }
  });

  const frameCount = clip.frames.length;
  const frameDuration = frameCount > 1
    ? (clip.frames[frameCount - 1].time - clip.frames[0].time) / (frameCount - 1)
    : 0;

  const features = clip.frames.map((frame, frameIndex) => extractFrameFeatures(frame, {
    horizons,
    tolerance,
    frameDuration,
    prevFrame: frameIndex > 0 ? clip.frames[frameIndex - 1] : null,
    nextFrame: frameIndex < frameCount - 1 ? clip.frames[frameIndex + 1] : null,
  }));

  return {
    id: clip.id,
    loop: clip.loop === true,
    frameCount,
    frames: clip.frames,
    features,
  };
}

export function buildMotionIndex(clips, config = {}) {
  if (!Array.isArray(clips)) {
    throw new TypeError('clips must be an array');
  }
  const normalizedConfig = normalizeConfig(config);
  const builtClips = [];
  const seenIds = new Set();
  for (const clip of clips) {
    const built = buildClipFeatures(clip, normalizedConfig.horizons, normalizedConfig.trajectoryTolerance);
    if (seenIds.has(built.id)) {
      throw new Error(`Duplicate clip id: ${built.id}`);
    }
    seenIds.add(built.id);
    builtClips.push(built);
  }

  return {
    config: normalizedConfig,
    clips: builtClips,
    clipIds: builtClips.map((clip) => clip.id),
    size: builtClips.reduce((total, clip) => total + clip.frameCount, 0),
  };
}

function groupCost(queryGroup, candidateGroup, scales) {
  let accumulated = 0;
  let comparedChannels = 0;
  for (let channel = 0; channel < queryGroup.values.length; channel += 1) {
    if (!queryGroup.mask[channel] || !candidateGroup.mask[channel]) continue;
    const scale = scales[queryGroup.dimensions[channel]];
    const normalizedDelta = (queryGroup.values[channel] - candidateGroup.values[channel]) / scale;
    const normalizedSquared = normalizedDelta * normalizedDelta;
    accumulated += normalizedSquared;
    comparedChannels += 1;
  }
  if (comparedChannels === 0) return { cost: null, comparedChannels };
  return { cost: accumulated / comparedChannels, comparedChannels };
}

function scoreCandidate(queryFeatures, candidateFeatures, config) {
  const groupResults = {};
  const missingGroups = [];
  let totalCost = 0;
  let comparableGroups = 0;

  for (const groupName of ['pose', 'motion', 'trajectory']) {
    const result = groupCost(
      queryFeatures[groupName],
      candidateFeatures[groupName],
      config.scales,
    );
    groupResults[groupName] = result.cost;
    if (result.cost === null) {
      missingGroups.push(groupName);
    } else {
      totalCost += config.weights[groupName] * result.cost;
      comparableGroups += 1;
    }
  }

  if (comparableGroups === 0) return null;
  return { costs: groupResults, missingGroups, totalCost };
}

export function circularFrameDistance(frameIndexA, frameIndexB, frameCount) {
  if (!Number.isInteger(frameCount) || frameCount <= 0) {
    throw new TypeError('frameCount must be a positive integer');
  }
  const direct = Math.abs(frameIndexA - frameIndexB);
  return Math.min(direct, frameCount - direct);
}

export function queryMotionIndex(index, queryFrame, options = {}) {
  const normalizedOptions = normalizeQueryOptions(options, index.config);
  validateFrame(queryFrame, 'queryFrame');
  const queryFeatures = extractFrameFeatures(queryFrame, {
    horizons: index.config.horizons,
    tolerance: index.config.trajectoryTolerance,
  });

  const candidates = [];
  for (const clip of index.clips) {
    if (normalizedOptions.excludedClipIds.has(clip.id)) continue;
    for (let frameIndex = 0; frameIndex < clip.frameCount; frameIndex += 1) {
      const score = scoreCandidate(queryFeatures, clip.features[frameIndex], index.config);
      if (score === null) continue;
      if (normalizedOptions.maxCost !== null && score.totalCost > normalizedOptions.maxCost) {
        continue;
      }
      const frame = clip.frames[frameIndex];
      candidates.push({
        clipId: clip.id,
        frameIndex,
        time: frame.time,
        loop: clip.loop,
        cost: score.totalCost,
        costs: score.costs,
        missingGroups: score.missingGroups,
      });
    }
  }

  candidates.sort((left, right) => {
    if (left.cost !== right.cost) return left.cost - right.cost;
    if (left.clipId !== right.clipId) return left.clipId < right.clipId ? -1 : 1;
    return left.frameIndex - right.frameIndex;
  });

  return candidates.slice(0, normalizedOptions.topK);
}
