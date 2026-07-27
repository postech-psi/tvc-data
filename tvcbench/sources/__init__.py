"""Acquisition sources. Each owns one device and emits `Sample`s on the master clock."""

from tvcbench.sources.base import Sample, Source, ThreadedSource

__all__ = ["Sample", "Source", "ThreadedSource"]
