/**
 * TopDownMap class
 */
class TopDownMap {
  // PUBLIC methods.

  /**
   * Create top-down map.
   * @param {PathFinder} pathFinder - pathFinder used to create map
   * @param {HTMLCanvasElement} canvas - canvas element for drawing map
   */
  constructor(pathFinder, canvas) {
    this.pathFinder = pathFinder;
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.imageData = this.createMap();
  }

  /**
   * Draw the topdown map to its canvas.
   */
  draw() {
    this.ctx.putImageData(this.imageData, 0, 0);
  }

  // PRIVATE methods.

  /*
   * Produces map using a regular grid sampling of navigability.
   */
  createMap() {
    const canvas = this.canvas;
    const bounds = this.pathFinder.bounds;
    let width = bounds.max[0] - bounds.min[0];
    let height = bounds.max[2] - bounds.min[2];
    let imageData = new ImageData(canvas.width, canvas.height);
    let scale = 1.0;
    let heightInPixels = 0;
    let widthInPixels = 0;
    let widthComponent = 0;
    let heightComponent = 2;

    // Best-Fit: orient and scale for tightest fit
    if (
      (width > height && canvas.width < canvas.height) ||
      (width < height && canvas.width > canvas.height)
    ) {
      widthComponent = 2;
      heightComponent = 0;
      let tmp = width;
      width = height;
      height = tmp;
    }
    if (height / width > canvas.height / canvas.width) {
      // Fit height
      scale = canvas.height / height;
      heightInPixels = canvas.height;
      widthInPixels = Math.round(width * scale - 0.5);
    } else {
      // Fit width
      scale = canvas.width / width;
      widthInPixels = canvas.width;
      heightInPixels = Math.round(height * scale - 0.5);
    }

    // Create map by painting navigable locations white
    const stride = 4 * canvas.width;
    const increment = 1 / scale;
    const color = 0xffffffff;
    let y = bounds.min[heightComponent] + increment / 2;
    let i, j;
    for (i = 0; i < heightInPixels; i++) {
      let x = bounds.min[widthComponent] + increment / 2;
      for (j = 0; j < widthInPixels; j++) {
        let point = [0, 0, 0];
        point[widthComponent] = x;
        point[heightComponent] = y;
        let isNavigable = this.pathFinder.isNavigable(point, 0.5);
        if (isNavigable) {
          imageData.data[i * stride + j * 4] = (color >> 24) & 0xff;
          imageData.data[i * stride + j * 4 + 1] = (color >> 16) & 0xff;
          imageData.data[i * stride + j * 4 + 2] = (color >> 8) & 0xff;
          imageData.data[i * stride + j * 4 + 3] = color & 0xff;
        }
        x += increment;
      }
      y += increment;
    }
    return imageData;
  }
}

export default TopDownMap;
