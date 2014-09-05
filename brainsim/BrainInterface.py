__author__ = 'GeorgHinkel'


class NeuronReference(object):
    """
    Represents a reference to one or multiple neurons
    """

    def __init__(self, label, n):
        self.__label = label
        self.__n = n

    def get_label(self):
        """
        Gets the label for this neuron reference
        """
        return self.__label

    label = property(get_label)

    def get_n(self):
        """
        Gets a number indicating how many neurons are referenced
        """
        return self.__n

    n = property(get_n)

    def __repr__(self):
        return self.__label + "(" + str(self.n) + ")"


class ISpikeGenerator(object):
    """
    Represents a communication object that may generate spikes
    """
    pass


class IFixedFrequencySpikeGenerator(ISpikeGenerator):
    """
    Represents a communication object that generates spikes on a fixed frequency
    """

    def __get_frequency(self):
        raise Exception("Not Implemented")

    def __set_frequency(self, value):
        raise Exception("Not Implemented")

    frequency = property(__get_frequency, __set_frequency)


class IPatternSpikeGenerator(ISpikeGenerator):
    """
    Represents a spike generator generating spikes in a pattern
    """

    def __get_pattern(self):
        raise Exception("Not Implemented")

    def __set_pattern(self, value):
        raise Exception("Not Implemented")

    pattern = property(__get_pattern, __set_pattern)


class IPoissonSpikeGenerator(ISpikeGenerator):
    """
    Represents a spike generator based on a Poisson Distribution
    """

    def __get_rate(self):
        raise Exception("Not Implemented")

    def __set_rate(self, rate):
        raise Exception("Not Implemented")

    rate = property(__get_rate, __set_rate)


class ISpikeDetector(object):
    """
    Represents a communication object that may detect spikes
    """
    pass


class ISpikeRecorder(ISpikeDetector):
    """
    Represents a communication object that records spikes from a certain neuron
    """

    def __get_recorded_spikes(self):
        raise Exception("Not Implemented")

    recorded_spikes = property(__get_recorded_spikes)


class INeuronVoltmeter(ISpikeDetector):
    """
    Represents a spike detector that integrates the spikes to the voltage of a neuron
    """

    def __get_voltage(self):
        raise Exception("Not Implemented")

    voltage = property(__get_voltage)


class IBrainCommunicationAdapter(object):
    """
    Represents the communication interface to the neuronal simulator
    """

    def __init__(self):
        self.__generated_neurons = {}
        self.__received_neurons = {}

    def register_generate_spikes(self, neurons, spike_generator_type):  # -> ISpikeGenerator:
        """
        Requests a communication object with the given spike generator type for the given set of neurons
        :param neurons: A reference to the neurons where the spike generator should be installed
        :param spike_generator_type: A spike generator type (see documentation for a list of allowed values)
        :return: A communication object
        """
        if not neurons in self.__generated_neurons:
            subscriber = self.create_spike_generator(neurons, spike_generator_type)
            self.__generated_neurons[neurons] = subscriber
            return subscriber
        else:
            return self.__generated_neurons[neurons]

    def register_consume_spikes(self, neurons, spike_detector_type):  # -> ISpikeDetector:
        """
        Requests a communication object with the given spike detector type for the given set of neurons
        :param neurons: A reference to the neurons where the spikes should be detected
        :param spike_detector_type: A spike detector type (see documentation for a list of allowed values)
        :return: A Communication object
        """
        if not neurons in self.__received_neurons:
            publisher = self.create_spike_detector(neurons, spike_detector_type)
            self.__received_neurons[neurons] = publisher
            return publisher
        else:
            return self.__received_neurons[neurons]

    def create_spike_generator(self, neurons, spike_generator_type):  # -> ISpikeGenerator:
        """
        Creates the spike generator for the given neurons and spike generator type
        :param neurons: The neuron reference where the generator should be created
        :param spike_generator_type: The generator type
        """
        raise Exception("Not Implemented")

    def create_spike_detector(self, neurons, spike_detector_type):  # -> ISpikeDetector:
        """
        Creates the spike detector for the given neurons and spike detector type
        :param neurons: The neuron reference where the detector should be created
        :param spike_detector_type: The detector type
        """
        raise Exception("Not Implemented")

    def initialize(self):
        """
        Initializes the adapter
        """
        raise Exception("Not Implemented")

    def refresh_buffers(self):
        """
        Refreshes all detector buffers
        """
        raise Exception("Not Implemented")


class IBrainControlAdapter(object):
    """
    Represents a controller object for the neuronal simulator
    """

    def is_alive(self):
        """
        Gets a status whether the neuronal simulator is still alive
        :return: True if the simulator is alive, otherwise False
        """
        raise Exception("Not Implemented")

    def initialize(self):
        """
        Initializes the neuronal simulator
        """
        raise Exception("Not Implemented")

    def run_step(self, dt):
        """
        Runs the neuronal simulator for the given amount of simulated time
        :param dt the simulated time
        """
        raise Exception("Not Implemented")

    def shutdown(self):
        """
        Shuts down the neuronal simulator
        """
        raise Exception("Not Implemented")