from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyServiceCaller
from flexbe_core.proxy import ProxyActionClient


class RefBoxParserState(EventState):
        '''
        Implements a state that can perform a calculation based on userdata.
        [...]
        
        -- calculation  function        The function that performs [...]

        ># input_value  object          Input to the calculation function.

        #> output_value object          The result of the calculation.

        <= done                         Indicates completion of the calculation.

        '''