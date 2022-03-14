import argparse
from re import L
import sys
import os
import os.path
import json

import torch
import torch.backends.cudnn
import torch.cuda
import torch.nn
import torch.utils.data
import wandb
from torchpack import distributed as dist
from torchpack.callbacks import Callbacks, SaverRestore
from torchpack.environ import auto_set_run_dir, set_run_dir
from torchpack.utils.config import configs
from torchpack.utils.logging import logger
from tqdm import tqdm

from core import builder
from core.callbacks import MeanIoU
from core.trainers import SemanticKITTITrainer
from model_zoo import minkunet, spvcnn, spvnas_specialized
from core.rosbag_to_pcl import RosbagToPCLExtractor

use_wandb = False
if use_wandb:
    wandb.init(project="my-test-project", entity="liamkboyle")

def main() -> None:
    dist.init()

    torch.backends.cudnn.benchmark = True
    torch.cuda.set_device(dist.local_rank())

    parser = argparse.ArgumentParser()
    parser.add_argument('config', metavar='FILE', help='config file')
    parser.add_argument('--run-dir', metavar='DIR', help='run directory')
    parser.add_argument('--name', type=str, help='model name')
    args, opts = parser.parse_known_args()

    configs.load(args.config, recursive=True)
    configs.update(opts)
    if args.run_dir is None:
        args.run_dir = auto_set_run_dir()
    else:
        set_run_dir(args.run_dir)

    logger.info(' '.join([sys.executable] + sys.argv))
    logger.info(f'Experiment started: "{args.run_dir}".' + '\n' + f'{configs}')
    print(os.getcwd())
    if len(os.listdir(configs.dataset.root)) == 0:
        print("Directory is empty")
        # Convert rosbag to .bin files
        extractor = RosbagToPCLExtractor(configs.dataset.rosbag, configs.dataset.topic)

    dataset = builder.make_dataset()
    dataflow = {}
    for split in dataset:
        sampler = torch.utils.data.distributed.DistributedSampler(
            dataset[split],
            num_replicas=dist.size(),
            rank=dist.rank(),
            shuffle=(split == 'train'))
        dataflow[split] = torch.utils.data.DataLoader(
            dataset[split],
            batch_size=configs.batch_size if split == 'train' else 1,
            sampler=sampler,
            num_workers=configs.workers_per_gpu,
            pin_memory=True,
            collate_fn=dataset[split].collate_fn)
    if 'spvnas' in args.name.lower():
        model = spvnas_specialized(args.name)
    elif 'spvcnn' in args.name.lower():
        model = spvcnn(args.name)
    elif 'mink' in args.name.lower():
        model = minkunet(args.name)
    else:
        raise NotImplementedError
    if use_wandb:
        wandb.watch(model)

    model = torch.nn.parallel.DistributedDataParallel(
        model.cuda(),
        device_ids=[dist.local_rank()],
        find_unused_parameters=True)
    model.eval()

    criterion = builder.make_criterion()
    optimizer = builder.make_optimizer(model)
    scheduler = builder.make_scheduler(optimizer)

    trainer = SemanticKITTITrainer(model=model,
                                  criterion=criterion,
                                  optimizer=optimizer,
                                  scheduler=scheduler,
                                  num_workers=configs.workers_per_gpu,
                                  seed=configs.train.seed)
    callbacks = Callbacks([
       SaverRestore(),
       MeanIoU(configs.data.num_classes, configs.data.ignore_label)
    ])
    callbacks._set_trainer(trainer)
    trainer.callbacks = callbacks
    trainer.dataflow = dataflow['test']

    trainer.before_train()
    trainer.before_epoch()

    model.eval()
    dataset['test']

    counter = 0
    for feed_dict in tqdm(dataflow['test'], desc='eval'):
        _inputs = {}
        for key, value in feed_dict.items():
            if 'name' not in key:
                _inputs[key] = value.cuda()

        inputs = _inputs['lidar']
        targets = feed_dict['targets'].F.long().cuda(non_blocking=True)
        outputs = model(inputs)
        point_cloud = feed_dict["lidar"].coords.cpu().numpy()
        plot_point_cloud = point_cloud
        plot_labels = []
        
        for idx, output in enumerate(outputs):
            plot_labels.append(output.argmax().item())
            if(output.argmax().item() > 5):
                point_cloud[idx, -1] = output.argmax().item() - 5
            else:
                point_cloud[idx, -1] = 0
        invs = feed_dict['inverse_map']
        all_labels = feed_dict['targets_mapped']
        _outputs = []
        _targets = []
        for idx in range(invs.C[:, -1].max() + 1):
            cur_scene_pts = (inputs.C[:, -1] == idx).cpu().numpy()
            cur_inv = invs.F[invs.C[:, -1] == idx].cpu().numpy()
            cur_label = (all_labels.C[:, -1] == idx).cpu().numpy()
            outputs_mapped = outputs[cur_scene_pts][cur_inv].argmax(1)
            targets_mapped = all_labels.F[cur_label]
            _outputs.append(outputs_mapped)
            _targets.append(targets_mapped)
        outputs = torch.cat(_outputs, 0)
        targets = torch.cat(_targets, 0)
        output_dict = {'outputs': outputs, 'targets': targets}
        trainer.after_step(output_dict)

        if use_wandb and not counter % 10:
            wandb.log({"point_cloud": wandb.Object3D(point_cloud)})

        # Save labelled point cloud every tenth iteration
        if not counter % 10 and configs.logging.save_data:
            plot_data = [
            {
                'name': 'my_point_cloud',
                'points': plot_point_cloud[:,0:3].tolist(),
                'lables': plot_labels
            }
            ]
            filename = str(counter) + ".json"
            filename = filename.zfill(9)
            plot_file_path = configs.dataset.output_dir + filename
            with open(plot_file_path, 'w') as fp:
                json.dump(plot_data, fp)
        counter += 1
        # if counter > 0:
        #     break

    results = trainer.after_epoch()
    print(results)


if __name__ == '__main__':
    main()
