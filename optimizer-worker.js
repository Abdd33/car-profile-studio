importScripts('engine.js');
self.onmessage = async ({data}) => {
  try {
    const response = await fetch('assets/model.json');
    if (!response.ok) throw Error('模型加载失败，请刷新后重试。');
    setup(await response.json());
    const result = optimize(data, generation => self.postMessage({generation}));
    self.postMessage({result});
  } catch (error) {
    self.postMessage({error:error.message});
  }
};
