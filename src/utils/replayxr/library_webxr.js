var LibraryWebXR = {

$WebXR: {
    _coordinateSystem: null,
    _curRAF: null,

    _nativize_vec3: function(offset, vec) {
        setValue(offset + 0, vec[0], 'float');
        setValue(offset + 4, vec[1], 'float');
        setValue(offset + 8, vec[2], 'float');

        return offset + 12;
    },

    _nativize_matrix: function(offset, mat) {
        for (var i = 0; i < 16; ++i) {
            setValue(offset + i*4, mat[i], 'float');
        }

        return offset + 16*4;
    },
    /* Sets input source values to offset and returns pointer after struct */
    _nativize_input_source: function(offset, inputSource, id) {
        var handedness = -1;
        if(inputSource.handedness == "left") handedness = 0;
        else if(inputSource.handedness == "right") handedness = 1;

        var targetRayMode = 0;
        if(inputSource.targetRayMode == "tracked-pointer") targetRayMode = 1;
        else if(inputSource.targetRayMode == "screen") targetRayMode = 2;

        setValue(offset, id, 'i32');
        offset +=4;
        setValue(offset, handedness, 'i32');
        offset +=4;
        setValue(offset, targetRayMode, 'i32');
        offset +=4;

        return offset;
    },

    _set_input_callback: function(event, callback, userData) {
        var s = Module['webxr_session'];
        if(!s) return;
        if(!callback) return;

        s.addEventListener(event, function(e) {
            /* Nativize input source */
            var inputSource = Module._malloc(8); // 2*sizeof(int32)
            WebXR._nativize_input_source(inputSource, e.inputSource, i);

            /* Call native callback */
            Module.dynCall('vii', callback, [inputSource, userData]);

            _free(inputSource);
        });
    },

    _set_session_callback: function(event, callback, userData) {
        var s = Module['webxr_session'];
        if(!s) return;
        if(!callback) return;

        s.addEventListener(event, function() {
            Module.dynCall('vi', callback, [userData]);
        });
    }
},

webxr_init: function(mode, frameCallback, startSessionCallback, endSessionCallback, errorCallback, userData) {
    function onError(errorCode) {
        if(!errorCallback) return;
        Module.dynCall('vii', errorCallback, [userData, errorCode]);
    };

    function onSessionEnd(session) {
        if(!endSessionCallback) return;
        Module.dynCall('vi', endSessionCallback, [userData]);
    };

    function onSessionStart() {
        if(!startSessionCallback) return;
        Module.dynCall('vi', startSessionCallback, [userData]);
    };

    function onFrame(time, frame) {
        if(!frameCallback) return;

        /* Request next frame */
        const session = frame.session;
        /* RAF is set to null on session end to avoid rendering */
        if(Module['webxr_session'] != null) session.requestAnimationFrame(onFrame);

        const pose = frame.getViewerPose(WebXR._coordinateSystem);
        if(!pose) return;

        const SIZE_OF_WEBXR_VIEW = (16 + 16 + 4)*4;
        const views = Module._malloc(SIZE_OF_WEBXR_VIEW*2 + 16*4);

        const glLayer = session.renderState.baseLayer;
        pose.views.forEach(function(view) {
            const viewport = glLayer.getViewport(view);
            const viewMatrix = view.transform.inverse.matrix;
            let offset = views + SIZE_OF_WEBXR_VIEW*(view.eye == 'left' ? 0 : 1);

            offset = WebXR._nativize_matrix(offset, viewMatrix);
            offset = WebXR._nativize_matrix(offset, view.projectionMatrix);

            setValue(offset + 0, viewport.x, 'i32');
            setValue(offset + 4, viewport.y, 'i32');
            setValue(offset + 8, viewport.width, 'i32');
            setValue(offset + 12, viewport.height, 'i32');
        });

        /* Model matrix */
        const modelMatrix = views + SIZE_OF_WEBXR_VIEW*2;
        WebXR._nativize_matrix(modelMatrix, pose.transform.matrix);

        Module.ctx.bindFramebuffer(Module.ctx.FRAMEBUFFER, glLayer.framebuffer);

        /* Set and reset environment for webxr_get_input_pose calls */
        Module['webxr_frame'] = frame;
        Module.dynCall('viiii', frameCallback, [userData, time, modelMatrix, views]);
        Module['webxr_frame'] = null;

        Module._free(views);
    };

    function onSessionStarted(session) {
        Module['webxr_session'] = session;

        // React to session ending
        session.addEventListener('end', function() {
            Module['webxr_session'].cancelAnimationFrame(WebXR._curRAF);
            WebXR._curRAF = null;
            Module['webxr_session'] = null;
            onSessionEnd();
        });

        // Give application a chance to react to session starting
        // e.g. finish current desktop frame.
        onSessionStart();

        // Ensure our context can handle WebXR rendering
        Module.ctx.makeXRCompatible().then(function() {
            // Create the base layer
            session.updateRenderState({
                baseLayer: new XRWebGLLayer(session, Module.ctx)
            });

            session.requestReferenceSpace('local').then(refSpace => {
                WebXR._coordinateSystem = refSpace;
                // Start rendering
                session.requestAnimationFrame(onFrame);
            });
        }, function(err) {
            onError(-3);
        });
    };

    if(navigator.xr) {
        // Check if XR session is supported
        navigator.xr.isSessionSupported((['inline', 'immersive-vr', 'immersive-ar'])[mode]).then(function() {
            Module['webxr_request_session_func'] = function() {
                navigator.xr.requestSession('immersive-vr').then(onSessionStarted);
            };
        }, function() {
            onError(-4);
        });
    } else {
        /* Call error callback with "WebXR not supported" */
        onError(-2);
    }
},

webxr_request_session: function() {
    var s = Module['webxr_request_session_func'];
    if(s) Module['webxr_request_session_func']();
},

webxr_request_exit: function() {
    var s = Module['webxr_session'];
    if(s) Module['webxr_session'].end();
},

webxr_set_projection_params: function(near, far) {
    var s = Module['webxr_session'];
    if(!s) return;

    s.depthNear = near;
    s.depthFar = far;
},

webxr_set_session_blur_callback: function(callback, userData) {
    WebXR._set_session_callback("blur", callback, userData);
},

webxr_set_session_focus_callback: function(callback, userData) {
    WebXR._set_session_callback("focus", callback, userData);
},

webxr_set_select_callback: function(callback, userData) {
    WebXR._set_input_callback("select", callback, userData);
},
webxr_set_select_start_callback: function(callback, userData) {
    WebXR._set_input_callback("selectstart", callback, userData);
},
webxr_set_select_end_callback: function(callback, userData) {
    WebXR._set_input_callback("selectend", callback, userData);
},

webxr_get_input_sources: function(outArrayPtr, max, outCountPtr) {
    var s = Module['webxr_session'];
    if(!s) return; // TODO(squareys) warning or return error

    var i = 0;
    for (let inputSource of s.inputSources) {
        if(i >= max) break;
        outArrayPtr = WebXR._nativize_input_source(outArrayPtr, inputSource, i);
        ++i;
    }
    setValue(outCountPtr, i, 'i32');
},

webxr_get_input_pose: function(source, outPosePtr) {
    var f = Module['webxr_frame'];
    if(!f) {
        console.warn("Cannot call webxr_get_input_pose outside of frame callback");
        return;
    }

    const id = getValue(source, 'i32');
    const input = Module['webxr_session'].inputSources[id];

    pose = f.getPose(input.gripSpace, WebXR._coordinateSystem);

    offset = outPosePtr;
    /* WebXRRay */
    offset = WebXR._nativize_matrix(offset, pose.transform.matrix);

    /* WebXRInputPose */
    //offset = WebXR._nativize_matrix(offset, pose.gripMatrix);
    //setValue(offset, pose.emulatedPosition, 'i32');
},

};

autoAddDeps(LibraryWebXR, '$WebXR');
mergeInto(LibraryManager.library, LibraryWebXR);
