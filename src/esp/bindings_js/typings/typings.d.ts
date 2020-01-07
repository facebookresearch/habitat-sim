export {};

type FSType = {
  createPreloadedFile(
    parent: String,
    file: String,
    url: String,
    canRead: Boolean,
    canWrite: Boolean
  ): void;
};

export interface ModuleType {
  preRun: Array<() => void>;
  onRuntimeInitialized: () => void;
  scene: String;
}

export interface WindowType {
  config: {
    scene: String;
    semantic: String;
  };
  vrEnabled: Boolean;
  viewerEnabled: Boolean;
}

declare global {
  const FS: FSType;
}
