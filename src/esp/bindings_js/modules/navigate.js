// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/*global Module */
import ObjectSensor from "./object_sensor";

const IOU_TOLERANCE = 0.5;

/**
 * NavigateTask class
 */
class NavigateTask {
  // PUBLIC methods.

  /**
   * Create navigate task.
   * @param {SimEnv} sim - simulator
   * @param {TopDownMap} topdown - TopDown Map
   * @param {Object} components - dictionary with status and canvas elements
   */
  constructor(sim, components) {
    this.sim = sim;
    this.components = components;
    this.topdown = components.topdown;
    this.semanticsEnabled = false;
    this.radarEnabled = false;
    this.score = 0;
    this.distance = 0.0;
    this.currentPosition = null;
    this.positions = [];

    if (this.components.semantic) {
      this.semanticsEnabled = true;
      this.semanticCtx = components.semantic.getContext("2d");
      this.semanticShape = this.sim.getObservationSpace("semantic").shape;
      this.semanticImageData = this.semanticCtx.createImageData(
        this.semanticShape.get(1),
        this.semanticShape.get(0)
      );
      this.semanticObservation = new Module.Observation();
      this.semanticScene = this.sim.sim.getSemanticScene();
      this.semanticObjects = this.semanticScene.objects;

      if (window.config.category) {
        document.getElementById("category").style.display = "block";
        document.getElementById("category").innerHTML =
          "find " + window.config.category;
        this.components.scope.style.display = "block";
        this.components.score.style.display = "block";
        const scopeWidth = this.components.scope.offsetWidth;
        const scopeHeight = this.components.scope.offsetHeight;
        const scopeInsetX = (this.components.canvas.width - scopeWidth) / 2;
        const scopeInsetY = (this.components.canvas.height - scopeHeight) / 2;
        const objectSearchRect = {
          left: scopeInsetX,
          top: scopeInsetY,
          right: scopeInsetX + scopeWidth,
          bottom: scopeInsetY + scopeHeight
        };
        this.objectSensor = new ObjectSensor(
          objectSearchRect,
          this.semanticShape,
          this.semanticScene,
          window.config.category
        );
      }

      components.canvas.onmousedown = e => {
        this.handleMouseDown(e);
      };
    }

    if (this.components.radar) {
      this.radarEnabled = true;
      this.radarCtx = components.radar.getContext("2d");
    }

    this.actions = [
      { name: "moveForward", key: "w" },
      { name: "turnLeft", key: "a" },
      { name: "turnRight", key: "d" },
      { name: "lookUp", key: "ArrowUp" },
      { name: "lookDown", key: "ArrowDown" }
    ];
  }

  updateScoreboard() {
    let topScores = [];
    const topScoresKey = window.config.category + "TopScores";
    if (localStorage[topScoresKey]) {
      topScores = JSON.parse(localStorage[topScoresKey]);
    }
    // Remove topScores with old schema
    if (topScores.length && topScores[0].score) {
      topScores = [];
    }
    let index;
    for (index = 0; index < topScores.length; index++) {
      if (topScores[index].distance > this.distance) {
        break;
      }
    }
    if (index <= 10) {
      const user = window.prompt("Enter your name for recording high score");
      topScores.splice(index, 0, { name: user, distance: this.distance });
      if (topScores.length > 10) {
        topScores.pop();
      }
    }
    let scoreHtml = "<H1>High Scores</H1><table>";
    scoreHtml += "<tr><th>Rank</th><th>Player</th><th>Distance (m)</th></tr>";
    for (let i = 0; i < topScores.length; i++) {
      const topScore = topScores[i];
      scoreHtml += "<tr>";
      scoreHtml += "<td>" + (i + 1) + "</td>";
      scoreHtml += "<td>" + topScore.name + "</td>";
      scoreHtml += "<td>" + topScore.distance.toFixed(2) + "</td>";
      scoreHtml += "</tr>";
    }
    scoreHtml += "</table>";
    this.components.scoreboard.style.visibility = "visible";
    this.components.scoreboard.innerHTML = scoreHtml;
    localStorage[topScoresKey] = JSON.stringify(topScores);
    setTimeout(() => {
      window.location = "index.html";
    }, 10000);
  }

  handleMouseDown(event) {
    const height = this.semanticShape.get(0);
    const width = this.semanticShape.get(1);
    const offsetY = height - 1 - event.offsetY; /* flip-Y */
    const offsetX = event.offsetX;
    const objectId = this.semanticData[width * offsetY + offsetX];
    const object = this.semanticObjects.get(objectId);
    if (object && object.category) {
      this.setStatus(object.category.getName(""));
    } else {
      this.setWarningStatus("unknown");
    }
  }

  /**
   * Initialize the task. Should be called once.
   */
  init() {
    this.bindKeys();
  }

  /**
   * Reset the task.
   */
  reset() {
    this.sim.reset();
    this.setStatus("Ready");
    this.render();
    this.currentPosition = this.sim.getAgentState().position;
    this.positions = [this.currentPosition];
  }

  // PRIVATE methods.

  updateScore(score) {
    this.components.score.innerHTML = "" + score.toFixed(2) + " m";
  }

  setStatus(text) {
    this.components.status.style = "color:white";
    this.components.status.innerHTML = text;
  }

  setErrorStatus(text) {
    this.components.status.style = "color:red";
    this.components.status.innerHTML = text;
  }

  setWarningStatus(text) {
    this.components.status.style = "color:orange";
    this.components.status.innerHTML = text;
  }

  setSuccessStatus(text) {
    this.components.status.style = "color:green";
    this.components.status.innerHTML = text;
  }

  renderImage() {
    this.sim.displayObservation("rgb");
    this.renderRadar();
  }

  renderSemanticImage() {
    if (!this.semanticsEnabled || this.semanticObjects.size() == 0) {
      return;
    }

    this.sim.getObservation("semantic", this.semanticObservation);
    const rawSemanticBuffer = this.semanticObservation.getData();
    const objectIds = new Uint32Array(
      rawSemanticBuffer.buffer,
      rawSemanticBuffer.byteOffset,
      rawSemanticBuffer.length / 4
    );
    this.semanticData = objectIds;

    // TOOD(msb) implement a better colorization scheme
    for (let i = 0; i < objectIds.length; i++) {
      const objectId = objectIds[i];
      if (objectId & 1) {
        this.semanticImageData.data[i * 4] = 255;
      } else {
        this.semanticImageData.data[i * 4] = 0;
      }
      if (objectId & 2) {
        this.semanticImageData.data[i * 4 + 1] = 255;
      } else {
        this.semanticImageData.data[i * 4 + 1] = 0;
      }
      if (objectId & 4) {
        this.semanticImageData.data[i * 4 + 2] = 255;
      } else {
        this.semanticImageData.data[i * 4 + 2] = 0;
      }
      this.semanticImageData.data[i * 4 + 3] = 255;
    }

    this.semanticCtx.putImageData(this.semanticImageData, 0, 0);
  }

  renderTopDown(options) {
    if (options.renderTopDown && this.topdown !== null) {
      this.topdown.moveTo(this.sim.getAgentState().position, 500);
    }
  }

  renderRadar() {
    if (!this.radarEnabled) {
      return;
    }
    const width = 100,
      height = 100;
    let radius = width / 2;
    let centerX = width / 2;
    let centerY = height / 2;
    let ctx = this.radarCtx;
    ctx.clearRect(0, 0, width, height);
    ctx.globalAlpha = 0.5;
    // draw circle
    ctx.fillStyle = "darkslategray";
    ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);
    ctx.fill();
    // draw sector
    ctx.fillStyle = "darkgray";
    ctx.beginPath();
    // TODO(msb) Currently 90 degress but should really use fov.
    ctx.arc(centerX, centerY, radius, (-Math.PI * 3) / 4, -Math.PI / 4);
    ctx.lineTo(centerX, centerY);
    ctx.closePath();
    ctx.fill();
    // draw target
    ctx.globalAlpha = 1.0;
    ctx.beginPath();
    let magnitude, angle;
    [magnitude, angle] = this.sim.distanceToGoal();
    let normalized = magnitude / (magnitude + 1);
    let targetX = centerX + Math.sin(angle) * radius * normalized;
    let targetY = centerY - Math.cos(angle) * radius * normalized;
    ctx.fillStyle = "maroon";
    ctx.arc(targetX, targetY, 3, 0, 2 * Math.PI);
    ctx.fill();
  }

  render() {
    this.renderImage();
    this.renderSemanticImage();
  }

  handleAction(action) {
    this.sim.step(action);
    //this.setStatus(action);
    this.render();
    ++this.score;
    //this.updateScore();
  }

  calculateDistanceTo(position) {
    const a = this.currentPosition;
    const delta = position.map((b, i) => b - a[i]);
    return Math.sqrt(delta.reduce((t, a) => t + a * a, 0));
  }

  bindKeys() {
    document.addEventListener(
      "keydown",
      event => {
        if (event.key === " " && this.objectSensor) {
          event.preventDefault();
          const iou = this.objectSensor.computeIOU(this.semanticData);
          if (iou > parseFloat(window.config.iou) * IOU_TOLERANCE) {
            this.setSuccessStatus(
              "You found the " + window.config.category + "!"
            );
            setTimeout(() => {
              this.updateScoreboard();
              this.topdown.drawPositions(this.positions);
              document.getElementById("topdown").style.display = "block";
            }, 1000);
          } else if (iou > 0) {
            this.setWarningStatus("IoU is too low. Please get a better view.");
          } else {
            this.setErrorStatus(window.config.category + " not found!");
          }
          return;
        }
        for (let a of this.actions) {
          if (event.key === a.key) {
            this.handleAction(a.name);
            event.preventDefault();
            break;
          }
        }
        if (event.key === "w") {
          const newPosition = this.sim.getAgentState().position;
          this.distance += this.calculateDistanceTo(newPosition);
          this.positions.push(newPosition);
          this.currentPosition = newPosition;
          this.updateScore(this.distance);
        }
      },
      true
    );
  }
}

export default NavigateTask;
