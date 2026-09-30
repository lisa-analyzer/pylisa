"""Structural replica of the ``random`` module (Python 3.14).

The real ``Random`` class inherits from the C-implemented ``_random.Random``
(which supplies ``random``, ``seed``, ``getstate``, ``setstate`` and
``getrandbits``); since ``_random`` is out of scope here, those methods are
declared directly on ``Random`` instead.
"""

from __future__ import annotations

from typing import Any, Iterable, Sequence, TypeVar

_T = TypeVar("_T")


class Random(object):
    VERSION: int

    def __init__(self, x: Any = None) -> None:
        # Initialize the generator, seeding it from x (or system entropy).
        pass

    def seed(self, a: Any = None, version: int = 2) -> None:
        # Reinitialize the generator's internal state from a.
        pass

    def getstate(self) -> tuple[Any, ...]:
        # Return an opaque object capturing the generator's internal state.
        pass

    def setstate(self, state: tuple[Any, ...]) -> None:
        # Restore the generator's internal state from a value returned by getstate().
        pass

    def getrandbits(self, k: int) -> int:
        # Return a non-negative integer with k random bits.
        pass

    def randbytes(self, n: int) -> bytes:
        # Return n random bytes.
        pass

    def randrange(self, start: int, stop: int | None = None, step: int = 1) -> int:
        # Return a randomly selected integer from range(start, stop, step).
        pass

    def randint(self, a: int, b: int) -> int:
        # Return a random integer N such that a <= N <= b.
        pass

    def choice(self, seq: Sequence[_T]) -> _T:
        # Return a random element from a non-empty sequence.
        pass

    def choices(
        self,
        population: Sequence[_T],
        weights: Sequence[float] | None = None,
        *,
        cum_weights: Sequence[float] | None = None,
        k: int = 1,
    ) -> list[_T]:
        # Return a k-sized list of elements chosen with replacement, optionally weighted.
        pass

    def shuffle(self, x: Sequence[Any]) -> None:
        # Shuffle the sequence x in place.
        pass

    def sample(self, population: Sequence[_T], k: int, *, counts: Iterable[int] | None = None) -> list[_T]:
        # Return a k-length list of unique elements chosen without replacement.
        pass

    def random(self) -> float:
        # Return the next random float in the range [0.0, 1.0).
        pass

    def uniform(self, a: float, b: float) -> float:
        # Return a random float N such that a <= N <= b.
        pass

    def triangular(self, low: float = 0.0, high: float = 1.0, mode: float | None = None) -> float:
        # Return a random float from a triangular distribution.
        pass

    def betavariate(self, alpha: float, beta: float) -> float:
        # Return a random float from the Beta distribution.
        pass

    def expovariate(self, lambd: float = 1.0) -> float:
        # Return a random float from the exponential distribution.
        pass

    def gammavariate(self, alpha: float, beta: float) -> float:
        # Return a random float from the Gamma distribution.
        pass

    def gauss(self, mu: float = 0.0, sigma: float = 1.0) -> float:
        # Return a random float from a Gaussian distribution (faster, non-thread-safe).
        pass

    def lognormvariate(self, mu: float, sigma: float) -> float:
        # Return a random float from a log-normal distribution.
        pass

    def normalvariate(self, mu: float = 0.0, sigma: float = 1.0) -> float:
        # Return a random float from a normal (Gaussian) distribution.
        pass

    def vonmisesvariate(self, mu: float, kappa: float) -> float:
        # Return a random float from the von Mises circular distribution.
        pass

    def paretovariate(self, alpha: float) -> float:
        # Return a random float from the Pareto distribution.
        pass

    def weibullvariate(self, alpha: float, beta: float) -> float:
        # Return a random float from the Weibull distribution.
        pass

    def binomialvariate(self, n: int = 1, p: float = 0.5) -> int:
        # Return a random integer from the binomial distribution.
        pass


class SystemRandom(Random):
    def random(self) -> float:
        # Return the next random float in [0.0, 1.0), sourced from os.urandom().
        pass

    def getrandbits(self, k: int) -> int:
        # Return a non-negative integer with k random bits, sourced from os.urandom().
        pass

    def randbytes(self, n: int) -> bytes:
        # Return n random bytes, sourced from os.urandom().
        pass

    def seed(self, *args: Any, **kwds: Any) -> None:
        # No-op: system randomness sources cannot be seeded.
        pass

    def getstate(self) -> Any:
        # Raise NotImplementedError: system randomness has no state to capture.
        pass

    def setstate(self, state: Any) -> Any:
        # Raise NotImplementedError: system randomness has no state to restore.
        pass


# ---------------------------------------------------------------------------
# Module-level convenience functions bound to a default Random() instance
# ---------------------------------------------------------------------------

def seed(a: Any = None, version: int = 2) -> None:
    # Reinitialize the default generator's internal state from a.
    pass

def getstate() -> tuple[Any, ...]:
    # Return an opaque object capturing the default generator's internal state.
    pass

def setstate(state: tuple[Any, ...]) -> None:
    # Restore the default generator's internal state.
    pass

def getrandbits(k: int) -> int:
    # Return a non-negative integer with k random bits.
    pass

def randbytes(n: int) -> bytes:
    # Return n random bytes.
    pass

def randrange(start: int, stop: int | None = None, step: int = 1) -> int:
    # Return a randomly selected integer from range(start, stop, step).
    pass

def randint(a: int, b: int) -> int:
    # Return a random integer N such that a <= N <= b.
    pass

def choice(seq: Sequence[_T]) -> _T:
    # Return a random element from a non-empty sequence.
    pass

def choices(
    population: Sequence[_T],
    weights: Sequence[float] | None = None,
    *,
    cum_weights: Sequence[float] | None = None,
    k: int = 1,
) -> list[_T]:
    # Return a k-sized list of elements chosen with replacement, optionally weighted.
    pass

def shuffle(x: Sequence[Any]) -> None:
    # Shuffle the sequence x in place.
    pass

def sample(population: Sequence[_T], k: int, *, counts: Iterable[int] | None = None) -> list[_T]:
    # Return a k-length list of unique elements chosen without replacement.
    pass

def random() -> float:
    # Return the next random float in the range [0.0, 1.0).
    pass

def uniform(a: float, b: float) -> float:
    # Return a random float N such that a <= N <= b.
    pass

def triangular(low: float = 0.0, high: float = 1.0, mode: float | None = None) -> float:
    # Return a random float from a triangular distribution.
    pass

def betavariate(alpha: float, beta: float) -> float:
    # Return a random float from the Beta distribution.
    pass

def expovariate(lambd: float = 1.0) -> float:
    # Return a random float from the exponential distribution.
    pass

def gammavariate(alpha: float, beta: float) -> float:
    # Return a random float from the Gamma distribution.
    pass

def gauss(mu: float = 0.0, sigma: float = 1.0) -> float:
    # Return a random float from a Gaussian distribution (faster, non-thread-safe).
    pass

def lognormvariate(mu: float, sigma: float) -> float:
    # Return a random float from a log-normal distribution.
    pass

def normalvariate(mu: float = 0.0, sigma: float = 1.0) -> float:
    # Return a random float from a normal (Gaussian) distribution.
    pass

def vonmisesvariate(mu: float, kappa: float) -> float:
    # Return a random float from the von Mises circular distribution.
    pass

def paretovariate(alpha: float) -> float:
    # Return a random float from the Pareto distribution.
    pass

def weibullvariate(alpha: float, beta: float) -> float:
    # Return a random float from the Weibull distribution.
    pass

def binomialvariate(n: int = 1, p: float = 0.5) -> int:
    # Return a random integer from the binomial distribution.
    pass
