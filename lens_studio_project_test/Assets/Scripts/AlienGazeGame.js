// Alien Gaze — a dependency-free AR target game for Spectacles and Preview.
// Look at the alien until the reticle locks on. Tap/click to fire instantly.

// @input Component.Camera camera
// @input SceneObject target
// @input SceneObject reticle
// @input Component.Text3D scoreText
// @input Component.Text3D timerText
// @input Component.Text3D messageText

var ROUND_SECONDS = 45.0;
var DWELL_SECONDS = 0.7;
var TARGET_LIFETIME = 3.2;
var RESPAWN_DELAY = 0.18;
var RESTART_DELAY = 4.0;
var HIT_RADIUS = 0.11;

var score = 0;
var timeLeft = ROUND_SECONDS;
var dwell = 0.0;
var targetAge = 0.0;
var respawnWait = 0.0;
var restartWait = 0.0;
var playing = true;
var rngState = 934857;
var targetBaseScale = new vec3(24.0, 24.0, 24.0);
var reticleBaseScale = new vec3(1.6, 1.6, 1.6);

function random01() {
    rngState = (rngState * 16807) % 2147483647;
    return (rngState - 1) / 2147483646;
}

function setMessage(text) {
    if (script.messageText) {
        script.messageText.text = text;
    }
}

function updateHud() {
    script.scoreText.text = "SCORE  " + score;
    script.timerText.text = "TIME  " + Math.max(0, Math.ceil(timeLeft));
}

function placeTarget() {
    // Stay inside the narrower Spectacles field of view, 135-180 cm away.
    var screenX = 0.32 + random01() * 0.36;
    var screenY = 0.32 + random01() * 0.30;
    var depth = 135.0 + random01() * 45.0;
    var worldPosition = script.camera.screenSpaceToWorldSpace(new vec2(screenX, screenY), depth);

    script.target.getTransform().setWorldPosition(worldPosition);
    script.target.getTransform().setLocalScale(targetBaseScale);
    script.target.enabled = true;
    targetAge = 0.0;
    dwell = 0.0;
    respawnWait = 0.0;
}

function hideAndQueueTarget() {
    script.target.enabled = false;
    respawnWait = RESPAWN_DELAY;
    dwell = 0.0;
}

function hitTarget() {
    if (!playing || !script.target.enabled) {
        return;
    }
    score += 1;
    timeLeft = Math.min(ROUND_SECONDS, timeLeft + 0.6);
    setMessage(score % 5 === 0 ? "COSMIC COMBO x" + score : "NICE SHOT!");
    updateHud();
    hideAndQueueTarget();
}

function isAimingAtTarget() {
    if (!script.target.enabled) {
        return false;
    }

    var worldPosition = script.target.getTransform().getWorldPosition();
    if (!script.camera.isSphereVisible(worldPosition, 18.0)) {
        return false;
    }

    var screenPosition = script.camera.worldSpaceToScreenSpace(worldPosition);
    var dx = screenPosition.x - 0.5;
    var dy = screenPosition.y - 0.5;
    return dx * dx + dy * dy <= HIT_RADIUS * HIT_RADIUS;
}

function startRound() {
    score = 0;
    timeLeft = ROUND_SECONDS;
    playing = true;
    restartWait = 0.0;
    setMessage("LOOK + HOLD TO ZAP");
    updateHud();
    placeTarget();
}

function finishRound() {
    playing = false;
    script.target.enabled = false;
    dwell = 0.0;
    restartWait = RESTART_DELAY;
    setMessage("FINAL SCORE  " + score + "\nNEW ROUND SOON");
    script.timerText.text = "TIME  0";
}

function onUpdate() {
    var dt = getDeltaTime();

    if (!playing) {
        restartWait -= dt;
        if (restartWait <= 0.0) {
            startRound();
        }
        return;
    }

    timeLeft -= dt;
    if (timeLeft <= 0.0) {
        timeLeft = 0.0;
        finishRound();
        return;
    }

    if (script.target.enabled) {
        targetAge += dt;
        if (targetAge >= TARGET_LIFETIME) {
            setMessage("ALIEN ESCAPED!");
            hideAndQueueTarget();
        } else if (isAimingAtTarget()) {
            dwell += dt;
            var lock = Math.min(1.0, dwell / DWELL_SECONDS);
            var pulse = 1.0 + lock * 0.32;
            script.reticle.getTransform().setLocalScale(reticleBaseScale.uniformScale(pulse));
            script.target.getTransform().setLocalScale(targetBaseScale.uniformScale(1.0 + lock * 0.08));
            setMessage(lock > 0.5 ? "LOCKING ON..." : "TARGET ACQUIRED");
            if (dwell >= DWELL_SECONDS) {
                hitTarget();
            }
        } else {
            dwell = 0.0;
            script.reticle.getTransform().setLocalScale(reticleBaseScale);
            script.target.getTransform().setLocalScale(targetBaseScale);
            if (targetAge > 0.75) {
                setMessage("LOOK + HOLD TO ZAP");
            }
        }
    } else {
        respawnWait -= dt;
        if (respawnWait <= 0.0) {
            placeTarget();
            setMessage("FIND THE ALIEN!");
        }
    }

    updateHud();
}

script.createEvent("OnStartEvent").bind(startRound);
script.createEvent("UpdateEvent").bind(onUpdate);
script.createEvent("TouchStartEvent").bind(function () {
    if (!playing) {
        startRound();
    } else if (isAimingAtTarget()) {
        hitTarget();
    }
});
