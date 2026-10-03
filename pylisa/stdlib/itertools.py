"""Structural replica of the ``itertools`` module (Python 3.14)."""

from __future__ import annotations

from typing import Any, Callable, Generic, Iterable, Iterator, TypeVar

_T = TypeVar("_T")
_S = TypeVar("_S")


# ---------------------------------------------------------------------------
# Infinite iterators
# ---------------------------------------------------------------------------

class count(Generic[_T], object):
    def __init__(self, start: _T = 0, step: Any = 1) -> None:
        # Create an iterator counting up from start by step, indefinitely.
        pass

    def __iter__(self) -> "count[_T]":
        # Return self, since count objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next value in the count sequence.
        pass


class cycle(Generic[_T], object):
    def __init__(self, iterable: Iterable[_T]) -> None:
        # Create an iterator that repeats the elements of iterable indefinitely.
        pass

    def __iter__(self) -> "cycle[_T]":
        # Return self, since cycle objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next value, restarting from the beginning when exhausted.
        pass


class repeat(Generic[_T], object):
    def __init__(self, object: _T, times: int | None = None) -> None:
        # Create an iterator that returns object over and over, times times (or forever).
        pass

    def __iter__(self) -> "repeat[_T]":
        # Return self, since repeat objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the repeated object, or raise StopIteration once exhausted.
        pass

    def __length_hint__(self) -> int:
        # Return the number of remaining repetitions.
        pass


# ---------------------------------------------------------------------------
# Iterators terminating on the shortest input sequence
# ---------------------------------------------------------------------------

class accumulate(Generic[_T], object):
    def __init__(self, iterable: Iterable[_T], func: Callable[[_T, _T], _T] | None = None, *, initial: _T | None = None) -> None:
        # Create an iterator of accumulated results of applying func (default: sum).
        pass

    def __iter__(self) -> "accumulate[_T]":
        # Return self, since accumulate objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next accumulated value.
        pass


class batched(Generic[_T], object):
    def __init__(self, iterable: Iterable[_T], n: int, *, strict: bool = False) -> None:
        # Create an iterator yielding successive n-sized tuples from iterable.
        pass

    def __iter__(self) -> "batched[_T]":
        # Return self, since batched objects are their own iterator.
        pass

    def __next__(self) -> tuple[_T, ...]:
        # Return the next batch of up to n elements.
        pass


class chain(Generic[_T], object):
    def __init__(self, *iterables: Iterable[_T]) -> None:
        # Create an iterator that chains together elements from multiple iterables.
        pass

    @classmethod
    def from_iterable(cls, iterable: Iterable[Iterable[_T]]) -> "chain[_T]":
        # Create a chain iterator from a single iterable of iterables.
        pass

    def __iter__(self) -> "chain[_T]":
        # Return self, since chain objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next element, moving to the next iterable when one is exhausted.
        pass


class compress(Generic[_T], object):
    def __init__(self, data: Iterable[_T], selectors: Iterable[Any]) -> None:
        # Create an iterator filtering data by the truthiness of selectors.
        pass

    def __iter__(self) -> "compress[_T]":
        # Return self, since compress objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next element of data whose corresponding selector is truthy.
        pass


class dropwhile(Generic[_T], object):
    def __init__(self, predicate: Callable[[_T], bool], iterable: Iterable[_T]) -> None:
        # Create an iterator that drops elements while predicate is true, then yields the rest.
        pass

    def __iter__(self) -> "dropwhile[_T]":
        # Return self, since dropwhile objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next element once the predicate has become false.
        pass


class filterfalse(Generic[_T], object):
    def __init__(self, predicate: Callable[[_T], bool] | None, iterable: Iterable[_T]) -> None:
        # Create an iterator yielding elements for which predicate is false.
        pass

    def __iter__(self) -> "filterfalse[_T]":
        # Return self, since filterfalse objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next element that fails the predicate.
        pass


class groupby(Generic[_T, _S], object):
    def __init__(self, iterable: Iterable[_T], key: Callable[[_T], _S] | None = None) -> None:
        # Create an iterator grouping consecutive elements sharing the same key.
        pass

    def __iter__(self) -> "groupby[_T, _S]":
        # Return self, since groupby objects are their own iterator.
        pass

    def __next__(self) -> tuple[_S, Iterator[_T]]:
        # Return the next (key, sub-iterator) group.
        pass


class islice(Generic[_T], object):
    def __init__(self, iterable: Iterable[_T], stop: int | None) -> None:
        # Create an iterator returning selected elements from iterable, like slicing.
        pass

    def __iter__(self) -> "islice[_T]":
        # Return self, since islice objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next selected element.
        pass


class pairwise(Generic[_T], object):
    def __init__(self, iterable: Iterable[_T]) -> None:
        # Create an iterator of consecutive overlapping pairs from iterable.
        pass

    def __iter__(self) -> "pairwise[_T]":
        # Return self, since pairwise objects are their own iterator.
        pass

    def __next__(self) -> tuple[_T, _T]:
        # Return the next (previous, current) pair.
        pass


class starmap(Generic[_T], object):
    def __init__(self, function: Callable[..., _T], iterable: Iterable[Iterable[Any]]) -> None:
        # Create an iterator applying function to argument tuples drawn from iterable.
        pass

    def __iter__(self) -> "starmap[_T]":
        # Return self, since starmap objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next result of function(*args) for the next argument tuple.
        pass


class takewhile(Generic[_T], object):
    def __init__(self, predicate: Callable[[_T], bool], iterable: Iterable[_T]) -> None:
        # Create an iterator yielding elements while predicate remains true.
        pass

    def __iter__(self) -> "takewhile[_T]":
        # Return self, since takewhile objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next element, stopping as soon as the predicate is false.
        pass


class _tee(Generic[_T], object):
    def __iter__(self) -> "_tee[_T]":
        # Return self, since tee objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next element of this independent copy of the source iterator.
        pass

    def __copy__(self) -> "_tee[_T]":
        # Return another independent copy sharing the same underlying buffer.
        pass


def tee(iterable: Iterable[_T], n: int = 2) -> tuple[_tee[_T], ...]:
    # Return n independent iterators over the same source iterable.
    pass


class zip_longest(Generic[_T], object):
    def __init__(self, *iterables: Iterable[Any], fillvalue: Any = None) -> None:
        # Create an iterator aggregating elements, padding shorter iterables with fillvalue.
        pass

    def __iter__(self) -> "zip_longest[_T]":
        # Return self, since zip_longest objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next tuple of aggregated elements.
        pass


# ---------------------------------------------------------------------------
# Combinatoric iterators
# ---------------------------------------------------------------------------

class product(Generic[_T], object):
    def __init__(self, *iterables: Iterable[Any], repeat: int = 1) -> None:
        # Create an iterator over the Cartesian product of the input iterables.
        pass

    def __iter__(self) -> "product[_T]":
        # Return self, since product objects are their own iterator.
        pass

    def __next__(self) -> _T:
        # Return the next tuple in the Cartesian product.
        pass


class permutations(Generic[_T], object):
    def __init__(self, iterable: Iterable[_T], r: int | None = None) -> None:
        # Create an iterator over successive r-length permutations of iterable.
        pass

    def __iter__(self) -> "permutations[_T]":
        # Return self, since permutations objects are their own iterator.
        pass

    def __next__(self) -> tuple[_T, ...]:
        # Return the next permutation tuple.
        pass


class combinations(Generic[_T], object):
    def __init__(self, iterable: Iterable[_T], r: int) -> None:
        # Create an iterator over r-length combinations of iterable, without repetition.
        pass

    def __iter__(self) -> "combinations[_T]":
        # Return self, since combinations objects are their own iterator.
        pass

    def __next__(self) -> tuple[_T, ...]:
        # Return the next combination tuple, in lexicographic order.
        pass


class combinations_with_replacement(Generic[_T], object):
    def __init__(self, iterable: Iterable[_T], r: int) -> None:
        # Create an iterator over r-length combinations of iterable, with repetition allowed.
        pass

    def __iter__(self) -> "combinations_with_replacement[_T]":
        # Return self, since combinations_with_replacement objects are their own iterator.
        pass

    def __next__(self) -> tuple[_T, ...]:
        # Return the next combination tuple, in lexicographic order.
        pass
