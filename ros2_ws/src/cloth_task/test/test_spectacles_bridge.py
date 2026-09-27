import os
import signal
import time

import pytest

from cloth_task.spectacles_bridge import install_stop_signals


@pytest.fixture
def ignored_stop_signals():
    """As a node started from a background job inherits them: SIGINT (and here SIGTERM)
    ignored."""
    saved = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    for s in saved:
        signal.signal(s, signal.SIG_IGN)
    yield
    for s, handler in saved.items():
        signal.signal(s, handler)


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM])
def test_stop_signals_interrupt_even_when_inherited_as_ignored(ignored_stop_signals, sig):
    install_stop_signals()
    with pytest.raises(KeyboardInterrupt):
        os.kill(os.getpid(), sig)
        time.sleep(1.0)  # the handler runs here, in the main thread
