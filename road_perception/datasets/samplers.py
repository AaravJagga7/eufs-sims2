"""Multitask batch mixer sampler (owned by Person B).

Day 11 requirement:
Each batch comes from one dataset, and datasets take turns (e.g. RDD -> BDD -> IDD -> speed bumps),
with pothole and hump datasets appearing about twice as often.
"""
import math
import random
from typing import Dict, Iterator, List, Optional
import torch
from torch.utils.data import BatchSampler, Dataset, Sampler

from datasets.contract import SEED


class MultitaskBatchSampler(Sampler[List[int]]):
    """Batches samples such that every batch comes from a single dataset.
    Datasets are interleaved according to sampling weights.

    Pothole and speedbump datasets appear with weight ~2, others with weight ~1.
    """

    def __init__(
        self,
        dataset_indices: Dict[str, List[int]],
        batch_size: int = 8,
        weights: Optional[Dict[str, float]] = None,
        drop_last: bool = False,
        shuffle: bool = True,
        seed: int = SEED,
    ):
        """
        dataset_indices: mapping of dataset name -> list of global sample indices
        batch_size: number of samples per batch
        weights: sampling frequency multiplier per dataset (default: rdd/speedbump=2, others=1)
        """
        self.dataset_indices = {k: list(v) for k, v in dataset_indices.items() if len(v) > 0}
        self.batch_size = batch_size
        self.drop_last = drop_last
        self.shuffle = shuffle
        self.rng = random.Random(seed)

        # Default weights: pothole & hump datasets appear twice as often
        if weights is None:
            self.weights = {}
            for name in self.dataset_indices:
                if any(x in name.lower() for x in ["rdd", "speedbump", "pothole", "hump"]):
                    self.weights[name] = 2.0
                else:
                    self.weights[name] = 1.0
        else:
            self.weights = {k: float(weights.get(k, 1.0)) for k in self.dataset_indices}

    def __iter__(self) -> Iterator[List[int]]:
        # Make a copy of indices per dataset and shuffle if needed
        dataset_pools = {}
        for name, indices in self.dataset_indices.items():
            pool = list(indices)
            if self.shuffle:
                self.rng.shuffle(pool)
            dataset_pools[name] = pool

        dataset_ptrs = {name: 0 for name in self.dataset_indices}

        # Determine weighted sequence of datasets
        dataset_names = list(self.dataset_indices.keys())
        norm_weights = [self.weights[name] for name in dataset_names]
        total_w = sum(norm_weights)
        probs = [w / total_w for w in norm_weights]

        # Total number of batches across all datasets
        total_batches = 0
        for name, indices in self.dataset_indices.items():
            n = len(indices)
            batches = n // self.batch_size if self.drop_last else math.ceil(n / self.batch_size)
            total_batches += int(batches * self.weights[name])

        for _ in range(total_batches):
            # Choose next dataset weighted by frequency
            chosen_name = self.rng.choices(dataset_names, weights=probs, k=1)[0]
            pool = dataset_pools[chosen_name]
            ptr = dataset_ptrs[chosen_name]

            if ptr + self.batch_size > len(pool):
                if self.shuffle:
                    self.rng.shuffle(pool)
                ptr = 0

            batch = pool[ptr : ptr + self.batch_size]
            dataset_ptrs[chosen_name] = ptr + len(batch)

            if len(batch) == self.batch_size or not self.drop_last:
                yield batch

    def __len__(self) -> int:
        total = 0
        for name, indices in self.dataset_indices.items():
            n = len(indices)
            batches = n // self.batch_size if self.drop_last else math.ceil(n / self.batch_size)
            total += int(batches * self.weights[name])
        return total


class ConcatMultiTaskDataset(Dataset):
    """Combines multiple MultiTaskDatasets into one dataset with index tracking for the sampler."""

    def __init__(self, datasets: Dict[str, Dataset]):
        self.datasets = datasets
        self.dataset_names = list(datasets.keys())
        self.dataset_indices: Dict[str, List[int]] = {}

        self._flat_items = []
        offset = 0
        for name, ds in datasets.items():
            indices = []
            for i in range(len(ds)):
                self._flat_items.append((name, i))
                indices.append(offset + i)
            self.dataset_indices[name] = indices
            offset += len(ds)

    def __len__(self) -> int:
        return len(self._flat_items)

    def __getitem__(self, idx: int):
        name, inner_idx = self._flat_items[idx]
        return self.datasets[name][inner_idx]

    def make_sampler(self, batch_size: int = 8, shuffle: bool = True) -> MultitaskBatchSampler:
        return MultitaskBatchSampler(
            self.dataset_indices,
            batch_size=batch_size,
            shuffle=shuffle,
        )
