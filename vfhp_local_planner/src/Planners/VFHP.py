#!/usr/bin/env python
# -*- coding: utf-8 -*-

r"""

Este módulo define el controlador para evasión mediante el
algoritmo VFH+, y constantes asociadas.

"""

import numpy as np
import math


########################################
###            Constantes           ####
########################################

# Active window array indexes
MAG = 0
BETA = 1
DIST2 = 2
ABDIST = 3
GAMA = 4

###                                 ####
########################################

class VConst(object):
    """Clase que contiene los parámetros del algoritmo.
    """

    def __init__(self):
        # Size of the full Grid
        self.GRID_SIZE = 125
        r"""int: Tamaño de la cuadrícula de certeza.
        """

        # Maximum certantinty value
        self.C_MAX = 20
        r"""int: Valor máximo de certeza.
        """

        # Resolution of each cell (in m)
        self.RESOLUTION = np.float_(0.04)
        r"""float: Resolución de cada celda (en m).
        """

        # Size of the active window that
        # travels with the robot
        self.WINDOW_SIZE = 25
        r"""int: Tamaño de la ventana activa.

        .. warning::
            La ventana activa debe tener un número impar de celdas.
        """

        assert self.WINDOW_SIZE%2 == 1, "Window should have an odd number of cells for better results"

        self.WINDOW_CENTER = self.WINDOW_SIZE/2
        r"""int: Índice de la celda central en la ventana activa.

        Se define automáticamente a partir de ``WINDOW_SIZE``.
        """

        self.HIST_SIZE = 72
        r"""int: Cantidad de sectores en el histograma polar.
        """

        # Size of each polar sector
        # in the polar histogram (in degrees)
        self.ALPHA = 2*math.pi/self.HIST_SIZE
        r"""float: Tamaño de cada sector del histograma polar (en radianes).
        Se define automáticamente a partir de ``HIST_SIZE``.
        """



        # Constants for virtual vector magnitude calculations
        # D_max^2 = 2*(ws-1/2)^2*R^2
        # A - B*D_max^2 = 1
        self.D_max2 = math.pow((self.WINDOW_SIZE-1)*self.RESOLUTION, 2)/2.0
        self.E = 2.0
        self.D = 1.0
        self.B = np.float_(10.0)
        r"""float: Constante :math:`b` de la ecuación de la magnitud del
        vector de obstáculos.
        """

        self.A = np.float_(1+self.B*self.D_max2)
        r"""float: Constante :math:`a` de la ecuación de la magnitud del
        vector de obstáculos. Se define a partir de ``B`` de modo que
        se cumpla :math:`a - b\cdot d_{max}^2 = 1`.
        """

        # Constants for obstacle enlargement
        # Robot radius
        self.R_ROB = 0.02
        r"""float: Radio del robot (en m).
        """

        # Minimum obstacle distance
        self.D_S   = 0.04
        r"""float: Distancia mínima de obstáculo (en m).

        """
        self.R_RS  = self.R_ROB + self.D_S
        r"""float: Radio de compensación de obstáculos (en m).
        """

        # Valley/Peak threshold
        self.T_LO = 3000.0
        """float: Valor de umbral inferior para valles en el histograma polar.
        """
        self.T_HI = 3500.0
        """float: Valor de umbral superior para valles en el histograma polar.
        """
        self.WIDE_V = self.HIST_SIZE/5
        """int: Tamaño mínimo de valle ancho.
        """
        self.V_MAX = 0.0628
        """float: Velocidad máxima del robot.
        """
        self.V_MIN = 0.0 #0.00628
        """float: Velocidad mínima del robot.
        """

        # Cost function constants
        # Recommended MU1 > MU2 + MU3
        # MU1 = target following cost
        # MU2 = sharp steering cost
        # MU3 = direction commiting cost
        self.MU1 = 6.0
        """float: Peso del costo de seguimiento.
        """
        self.MU2 = 2.0
        """float: Peso del costo de cambios abruptos de dirección.
        """
        self.MU3 = 2.0
        """float: Peso del costo de compromiso a una dirección.
        """
        self.MAX_COST = math.pi*(self.MU1+self.MU2+self.MU3)
        """float: Máximo valor de la función de costo.
        """

        self.DIST_FCN = "GAUSS"
        """str: Función de magnitud para vectores de obstáculos, puede ser GAUSS o LINEAR.
        """

    def update(self):
        """Actualiza los parámetros cuyo valor depende de otros parámetros
        """

        assert self.WINDOW_SIZE%2 == 1, "Window should have an odd number of cells"
        self.WINDOW_CENTER = self.WINDOW_SIZE/2
        # assert 360 % self.ALPHA == 0, "Alpha must define an int amount of sectors"
        self.ALPHA = 2*math.pi/self.HIST_SIZE
        self.D_max2 = math.pow((self.WINDOW_SIZE-1)*self.RESOLUTION, 2)/2.0
        self.A = np.float_(1+self.B*self.D_max2)
        self.R_RS  = self.R_ROB + self.D_S
        self.WIDE_V = self.HIST_SIZE/5
        self.MAX_COST = math.pi*(self.MU1+self.MU2+self.MU3)


class VFHPModel(object):
    r"""Clase que define el controlador para evasión mediante
    el algoritmo VFH+.

    Attributes
    ----------
    obstacle_grid : ndarray of short
        La cuadrícula de certeza. Cada celda tiene un valor entre
        0 y :const:`C_MAX` que indica que tan probable es que
        haya un obstáculo ocupando la celda.

    active_window : ndarray of float
        La ventana activa que se mueve con el robot. Cada celda
        contiene cuatro valores: la magnitud del vector de
        obstaculos :math:`m_{i,j}`, la dirección del vector
        :math:`\beta_{i,j}`, la distancia al robot
        :math:`d_{i,j}` y el ángulo de ensanchamiento
        :math:`\gamma_{i,j}` .

    polar_hist : ndarray of float
        El histograma polar. Indica la densidad de obstáculos
        en cada sector alrededor de la vecindad del robot.

    bin_polar_hist : ndarray of bool
        El histograma polar binario. Se construye a partir del
        histograma polar. Un valor ``True`` indica un sector
        bloqueado.

    masked_polar_hist : ndarray of bool
        El histograma polar mascarado. Se contruye a partir del
        histograma polar binario y los radios de giro
        instantáneos. Un valor ``True`` indica un sector
        bloqueado.

    valleys : list of 2-tuples
        Contiene una lista de los valles en el histograma polar.
        Cada es un par :math:`(s_1,s_2)`, donde :math:`s_1` es
        el sector donde inicia y :math:`s_2` donde termina, en
        sentido antihorario.

    x_0 : float
        Posición del robot sobre el eje :math:`x`.
    y_0 : float
        Posición del robot sobre el eje :math:`y`.
    cita : float
        Orientación del robot resepecto el eje :math:`z`.
    vcita: float
        Dirección del movimiento respecto al eje :math:`z`.
    rotation_matrix : ndarray
        Matriz de rotacion del robot.

    i_0 : float
        Índice de la celda que ocupa el robot sobre la cuadrícula
        de certeza en el eje :math:`x`.
    j_0 : float
        Índice de la celda que ocupa el robot sobre la cuadrícula
        de certeza en el eje :math:`y`.
    k_0 : float
        Sector en el histrograma polar de la dirección del robot
        resepecto el eje :math:`z`.

    target : tuple
        Punto :math:`\left(x,\:y\right)` objetivo o ``None`` si
        no se está siguiendo una trayectoria.

    prev_dir : float
        Última dirección de control.
    prev_cost : cost
        Costo de la última dirección de control.



    Examples
    --------

    >>> import VFHP
    >>> pseudo_readings = np.float_([[0.3, np.radians(x)] for x in range(0,90,1)])
    >>> R_ROB = 0.01
    >>> robot = VFHP.VFHModel()
    >>> robot.update_position(1.5,1.5,270.0)
    >>> robot.update_obstacle_density(pseudo_readings)
    >>> robot.update_active_window()
    >>> robot.update_polar_histogram()
    >>> robot.update_bin_polar_histogram()
    >>> robot.update_masked_polar_hist(R_ROB,R_ROB)
    >>> valles = robot.find_valleys()
    >>> cita, v = robot.calculate_steering_dir(valles)

    """

    def __init__(self, const=None, plotter=False):

        if const is None or type(const) is not VConst:
            self.const = VConst()
            print "Using default parameters"
        else:
            self.const = const

        self.const.update()

        if plotter:
            self.plotter = Plotter()
        else:
            self.plotter = None

        print "INITIALIZING"

        if self.const.DIST_FCN == "GAUSS":
            print "Using gaussian dist func"
            print "B=%.3f, D=%.3f, E=%.3f" % (self.const.B, self.const.D, self.const.E)

        self._reset_struct()
        self._reset_attr()
        self._calc_active_window_params()

        print "PARAMS DONE"
        # print self.active_window[:,:,GAMA]


    def _calc_active_window_params(self):
        centro = self.const.WINDOW_SIZE*self.const.RESOLUTION/2.0
        # print self.const.WINDOW_SIZE
        for i in xrange(self.const.WINDOW_SIZE):
            for j in xrange(self.const.WINDOW_SIZE):
                if j == self.const.WINDOW_CENTER and i == self.const.WINDOW_CENTER:
                    continue
                celda_x = (i + 0.5)*self.const.RESOLUTION
                celda_y = (j + 0.5)*self.const.RESOLUTION
                beta_p = math.atan2(celda_y-centro, celda_x-centro)
                self.active_window[i,j,BETA] = beta_p
                # The distance is measured in terms of cells
                # (independent of scale/resolution of the grid)
                dist2 = (celda_x - centro)**2.0 + (celda_y - centro)**2.0
                self.active_window[i,j,DIST2] = dist2
                dist = math.sqrt(dist2)
                # self.active_window[i,j,ABDIST] = self.const.A - self.const.B*dist2
                # self.active_window[i,j,ABDIST] = math.exp(-math.pow(math.sqrt(dist2/self.const.D_max2), 2.0*self.const.E)/self.const.B)


                if self.const.DIST_FCN == "GAUSS":
                    #self.active_window[i,j,ABDIST] = math.exp(-math.pow(dist2/self.const.D_max2, self.const.E)/self.const.B)
                    self.active_window[i,j,ABDIST] = math.exp(-math.pow(dist/self.const.D, self.const.E)/self.const.B)
                elif self.const.DIST_FCN == "LINEAR":
                    self.active_window[i,j,ABDIST] = self.const.A - self.const.B*dist2
                else:
                    raise Exception("Invalid parameter for distance function: {} (must be LINEAR or GAUSS)".format(self.const.DIST_FCN))

                    #self.active_window[i,j,ABDIST] = self.const.A - self.const.B*dist2

                # print "(x,y) = ({:d},{:d})".format(i, j)
                # print "beta: {:.2f}".format(beta_p)
                # print "dist2: {:.2f}".format(dist2)
                if self.const.R_RS > math.sqrt(dist2):
                    self.active_window[i,j,GAMA] = math.pi/2.0
                    # print "Setting to 90 degrees"
                else:
                    gama = math.asin(self.const.R_RS/math.sqrt(dist2))
                    self.active_window[i,j,GAMA] = gama
                    assert gama >= 0, "Unexpected gama for cell ({:d},{:d}), r_rs={:.2f}, dist^2={:.2f}, gama={:.2f}".format(i,j,self.const.R_RS, dist2)
                    # print "Setting to {:.2f}".format(gama)


    def _reset_struct(self):
        # The Obstacle Grid keeps count of the certainty values
        # for each cell in the grid
        grid_dim = (self.const.GRID_SIZE, self.const.GRID_SIZE)
        self.obstacle_grid = np.zeros( grid_dim, dtype = np.int8 )

        # The Active Window has information (magnitude,
        # direction, distance to robot) of an obstacle vector
        # for each active cell in the grid [mij, betaij, d^2, a-bd^2_ij, gij]
        window_dim = (self.const.WINDOW_SIZE, self.const.WINDOW_SIZE, 5)
        self.active_window = np.zeros( window_dim, dtype = np.float_ )

        # The Polar Histogram maps each cell in the active window
        # to one or more angular sector
        hist_dim = self.const.HIST_SIZE
        self.polar_hist = np.zeros( hist_dim, dtype = np.float_ )

        # The Binary Polar Histogram defines free and blocked sectors
        self.bin_polar_hist = np.zeros( hist_dim, dtype = np.bool_ )

        # The Masked Polar Histogram further restricts the free sectors
        # depending on vehicle dynamics
        self.masked_polar_hist = np.zeros( hist_dim, dtype = np.bool_ )

        # The valleys are stored as start-end pairs of sectors
        # in the filtered polar histogram
        self.valleys = []

    def _reset_attr(self):

        # Real position of the robot
        self.x_0 = 0.0
        self.y_0 = 0.0
        self.cita = 0.0
        self.vcita = 0.0
        self.rotation_matrix = np.array([[1, 0],
        [0, 1]], dtype=np.float_)

        # Cell of the robot
        self.i_0 = 0
        self.j_0 = 0
        # Sector of the robot
        self.k_0 = 0

        self.target = None
        self.prev_dir = 0
        self.prev_cost = 0

    def set_target(self, x=None, y=None):
        r"""Define el punto objetivo o deshabilita el seguimiento
        de trayectorias.

        Define un punto objetivo :math:`(x,y)` para la evasión
        con seguimiento de trayectorias, o deshabilita el
        seguimiento si el método es invocado sin parámetros.

        Parameters
        ----------
        x : float, opcional
            Posición absoluta del objetivo sobre el eje :math:`x`.
        y : float, opcional
            Posición absoluta del objetivo sobre el eje :math:`y`.

        Returns
        -------
        int
            Retorna 1 si se estableció un punto objetivo o 0
            si inhabilitó el seguimiento.

        Raises
        ------
        ValueError
            Si el objetivo dado se sale de la cuadrícula de
            certeza.

        Notes
        -----

        Modifica los siguientes atributos de clase:
         * :attr:`target`

        """
        if x != None and y != None:
            if x >= 0 and y >= 0 and \
                    x < self.const.GRID_SIZE*self.const.RESOLUTION and \
                    y < self.const.GRID_SIZE*self.const.RESOLUTION:
                self.target = (x,y)
                return 1
            else:
                raise ValueError("Tried to set the target ({:d},{:d}) outside the obstacle grid".format(x,y))
        else:
            self.target = None
            return 0

    def get_target_dist(self):
        if self.target:
            return math.sqrt((self.x_0-self.target[0])**2 + (self.y_0-self.target[1])**2)
        else:
            return 0

    def update_position(self, x, y, cita, vcita=None):
        r"""Actualiza la posición del robot.

        Parameters
        ----------
        x : float
            Posición absoluta del robot sobre el eje :math:`x`.
        y : float
            Posición absoluta del robot sobre el eje :math:`y`.
        cita : float
            Orientación del robot resepecto el eje :math:`z`
            en radianes.
        vcita : float, optional
            Dirección del movimiento del robot resepecto el eje :math:`z`
            en radianes. Si no se indica se asume que es igual a :param:`cita`

        Notes
        -----

        Modifica los siguientes atributos de clase:

        .. hlist::

            * :attr:`x_0`
            * :attr:`y_0`
            * :attr:`cita`
            * :attr:`vcita`
            * :attr:`i_0`
            * :attr:`j_0`
            * :attr:`k_0`
            * :attr:`rotation_matrix`

        """
        self.x_0 = x
        self.y_0 = y

        if cita > math.pi:
            cita -= 2*math.pi
        if cita < -math.pi:
            cita += 2*math.pi

        self.cita = cita
        radians = cita
        self.rotation_matrix = np.array([[math.cos(radians), -math.sin(radians)],
                                         [math.sin(radians), math.cos(radians)]])

        if vcita is None:
            self.vcita = cita
        else:
            if vcita > math.pi:
                vcita -= 2*math.pi
            if vcita < -math.pi:
                vcita += 2*math.pi
            self.vcita = vcita

        self.i_0 = int(x / self.const.RESOLUTION)
        self.j_0 = int(y / self.const.RESOLUTION)
        self.k_0 = int(self.cita / self.const.ALPHA)%self.const.HIST_SIZE


    def update_obstacle_density(self, sensor_readings, translation=np.zeros(2,dtype=np.float_)):
        r"""Actualiza la cuadrícula de certeza a partir de las
        lecturas de un sensor.

        Para cada lectura aumenta el valor de una única celda
        en 1, hasta un máximo de :const:`C_MAX`.

        Parameters
        ----------
        sensor_readings : ndarray
            Una estructura de datos tipo ``numpy.ndarray`` que
            contiene las lecturas de un sensor de distancias.
            Debe ser un arreglo de dimensiones
            :math:`(n\times 2)` para :math:`n` puntos o
            lecturas, donde cada par representa una coordenada
            polar :math:`(r,\theta)` respecto al marco de
            referencia del robot.

        translation : ndarray
            Una traslación (x, y) que representa la posición del sensor
            respecto al marco de referencia del robot.

        Raises
        ------
        TypeError
            Si ``sensor_readings`` no es de tipo
            ``numpy.ndarray``.
        ValueError
            Si ``sensor_readings`` no tiene las dimensiones
            correctas.

        Notes
        -----
        Las coordenadas deben ser dadas en metros y radianes.

        Modifica los siguientes atributos de clase:
         * :attr:`obstacle_grid`

        """
        # Receives a numpy array of (r, theta) data points #
        # r in meters, theta in radians
        # 0,0 means no reading

        if (type(sensor_readings) != np.ndarray):
            raise TypeError("Expected numpy ndarray, received %s" % type(sensor_readings))
        elif sensor_readings.ndim != 2 or sensor_readings.shape[1] != 2:
            raise ValueError("Expected (n, 2) array, received %s" % str(sensor_readings.shape))

        #valid_readings = sensor_readings[sensor_readings[:,0] > 0.0]
        reading_translation = np.matmul(self.rotation_matrix, translation)

        # print "Current theta: %.3f (rad), %.1f (deg)" % (self.cita, self.cita * 180 / math.pi)
        i = np.int_( (self.x_0 + reading_translation[0] + sensor_readings[:, 0]*np.cos(sensor_readings[:, 1] + self.cita))/self.const.RESOLUTION )
        j = np.int_( (self.y_0 + reading_translation[1] + sensor_readings[:, 0]*np.sin(sensor_readings[:, 1] + self.cita))/self.const.RESOLUTION )

        for x in xrange(sensor_readings.shape[0]):
            if self.obstacle_grid[i[x],j[x]] < self.const.C_MAX: self.obstacle_grid[i[x], j[x]] += 1

    def update_obstacle_density3(self, sensor_readings):
        if (type(sensor_readings) != np.ndarray):
            raise TypeError("Expected numpy ndarray, received %s" % type(sensor_readings))
        elif sensor_readings.ndim != 2 or sensor_readings.shape[1] != 2:
            raise ValueError("Expected (n, 2) array, received %s" % str(sensor_readings.shape))

        for x in xrange(sensor_readings.shape[0]):
            r, theta = sensor_readings[x,:]
            if r == 0 and theta == 0:
                continue
            i = int( (self.x_0 + r*math.cos(theta + self.cita))/self.const.RESOLUTION )
            j = int( (self.y_0 + r*math.sin(theta + self.cita))/self.const.RESOLUTION )
            if self.obstacle_grid[i,j] < self.const.C_MAX: self.obstacle_grid[i,j] += 1

    def decay_active_window(self, decay=1, guardband=0):
        r""" Decrementa en la certeza de las celdas de la ventana activa.

        Se decrementan únicamente las celdas mayores a cero.

        Parameters
        ----------
        decay : int
            Indica el decremento a aplicar.

        """
        i_window = max(self.i_0 - (self.const.WINDOW_CENTER + guardband), 0)
        j_window = max(self.j_0 - (self.const.WINDOW_CENTER + guardband), 0)
        i_max = min(i_window + self.const.WINDOW_SIZE + 2*guardband, self.const.GRID_SIZE)
        j_max = min(j_window + self.const.WINDOW_SIZE + 2*guardband, self.const.GRID_SIZE)

        # NOTE:
        # Entiéndase ->
        # A los valores de la ventana activa ([i_window:i_max, j_window:j_max])
        # Cuyo valor es mayor a decay, réstele decay.
        self.obstacle_grid[i_window:i_max, j_window:j_max] -= decay
        self.obstacle_grid[i_window:i_max, j_window:j_max][self.obstacle_grid[i_window:i_max, j_window:j_max] < 0] = 0

    def _active_grid(self):

        i_window = max(self.i_0 - (self.const.WINDOW_CENTER), 0)
        j_window = max(self.j_0 - (self.const.WINDOW_CENTER), 0)
        i_max = min(i_window + self.const.WINDOW_SIZE, self.const.GRID_SIZE)
        j_max = min(j_window + self.const.WINDOW_SIZE, self.const.GRID_SIZE)

        return self.obstacle_grid[i_window:i_max, j_window:j_max]

    def update_active_window(self):
        r"""Calcula la magnitud de los vectores de obstáculo para
        las celdas de la ventana activa.

        El cálculo se hace a partir de los datos guardados en la
        :attr:`cuadrícula de certeza<obstacle_grid>`.

        Notes
        -----
        Modifica los siguientes atributos de clase:
         * :attr:`active_window`

        """

        i_s = self.i_0 - (self.const.WINDOW_CENTER)
        j_s = self.j_0 - (self.const.WINDOW_CENTER)
        i_f = i_s + self.const.WINDOW_SIZE
        j_f = j_s + self.const.WINDOW_SIZE

        c2 = np.square(np.float_(self.obstacle_grid[i_s:i_f,j_s:j_f]))
        self.active_window[:,:,MAG] = np.multiply(c2,self.active_window[:,:,ABDIST])

        # print self._active_grid()
        # print c2

    def update_polar_histogram(self):
        r"""Actualiza el histograma polar de obstáculos.

        El cálculo se hace a partir de los vectores de obstáculo
        guardados en la :attr:`ventana activa<active_window>`.
        Cada celda puede afectar más de un sector angular,
        esto se determina mediante el radio de compensación
        :const:`R_RS`.

        Notes
        -----
        Modifica los siguientes atributos de clase:
         * :attr:`polar_hist`

        """

        # TODO: vectorizar, calcular k_range en el startup
        self.polar_hist[:] = 0.0

        for i in xrange(self.const.WINDOW_SIZE):
            for j in xrange(self.const.WINDOW_SIZE):
                if j == self.const.WINDOW_CENTER and i == self.const.WINDOW_CENTER:
                    continue

                low = int(math.floor((self.active_window[i,j,BETA] - self.active_window[i,j,GAMA])/self.const.ALPHA))
                high = int(math.ceil((self.active_window[i,j,BETA] + self.active_window[i,j,GAMA])/self.const.ALPHA))

                #k_range = [x%self.const.HIST_SIZE for x in np.linspace(low, high, high-low+1, True, dtype = int)]
                k_range = np.arange(low, high+1, 1, dtype=np.int_)%self.const.HIST_SIZE


                #print "DEBUG: celda (%d, %d)" % (i, j)
                #print k_range

                self.polar_hist[k_range] += self.active_window[i, j, MAG]

        #return self.polar_hist

    def update_bin_polar_histogram(self):
        r"""Calcula el histograma polar binario.

        El cálculo se hace a partir del
        :attr:`histograma polar<polar_hist>` y los valores
        de umbral :const:`T_HI` y :const:`T_LO`. Cada sector
        se indica como bloqueado (``True``) o libre (``False``).

        Modifica los siguientes atributos de clase:
         * :attr:`bin_polar_hist`

        """

        self.bin_polar_hist[self.polar_hist > self.const.T_HI] = True
        self.bin_polar_hist[self.polar_hist < self.const.T_LO] = False

        # for k in xrange(HIST_SIZE):
        #     if self.polar_hist[k] > T_HI:
        #         blocked = True
        #     elif self.polar_hist[k] < T_LO:
        #         blocked = False
        #     else:
        #         blocked = self.bin_polar_hist[k]
        #
        #     self.bin_polar_hist[k] = blocked

    def update_masked_polar_hist(self,steer_l,steer_r, omni=False):
        r"""Calcula el histograma polar mascarado.

        El cálculo se hace a partir del
        :attr:`histograma polar binario<bin_polar_hist>` y
        el radio de giro mínimo en el momento del cálculo.
        Las celdas ocupadas de la ventana activa bloquean
        sectores adicionales dependiendo de la dirección actual
        del robot y su capacidad de giro.

        Parameters
        ----------
        steer_l : float
            Radio de giro mínimo del robot hacia la izquierda.
        steer_r : float
            Radio de giro mínimo del robot hacia la derecha.

        Notes
        -----
        Modifica los siguientes atributos de clase:
         * :attr:`masked_polar_hist`
        """

        # if omni:
        #     self.masked_polar_hist[:] = self.bin_polar_hist[:]
        #     return
        # else:
        #     self.masked_polar_hist[:] = self.bin_polar_hist[:]
        #     return


        # FIXME
        # Pasar a radianes
        phi_back = self.vcita + math.pi
        phi_back = phi_back - 2 * math.pi if phi_back > math.pi else phi_back

        assert -math.pi <= self.vcita <= math.pi, 'vcita ({:.4f}) is not in range [-pi, pi]'.format(start)
        assert -math.pi <= phi_back <= math.pi, 'phi_back ({:.4f}) is not in range [-pi, pi]'.format(start)

        phi_left = phi_back
        phi_right = phi_back

        X_r = self.const.WINDOW_SIZE*self.const.RESOLUTION/2.0
        Y_r = self.const.WINDOW_SIZE*self.const.RESOLUTION/2.0

        print "INFO: Current dir is {:.3f}".format(self.vcita*180/math.pi)
        print "INFO: phi_back is {:.3f}".format(phi_back*180/math.pi)
        print "INFO: Center pos is ({:.3f}, {:.3f})".format(X_r, Y_r)

        active_grid = self._active_grid()

        #### Left steering radius
        r_robl_x = steer_l*math.cos(self.vcita+math.pi/2)
        r_robl_y = steer_l*math.sin(self.vcita+math.pi/2)
        # center of the steering radius in the active window
        l_steer_x = X_r + r_robl_x
        l_steer_y = Y_r + r_robl_y

        print "INFO: left steer center and radius: ({:.2f}, {:.2f}), {:.2f}".format(l_steer_x, l_steer_y, steer_l)

        #### Right steering radius
        r_robr_x = steer_r*math.cos(self.vcita-math.pi/2)
        r_robr_y = steer_r*math.sin(self.vcita-math.pi/2)
        # center of the steering radius in the active window
        r_steer_x = X_r + r_robr_x
        r_steer_y = Y_r + r_robr_y

        print "INFO: right steer center and radius: ({:.2f}, {:.2f}), {:.2f}".format(r_steer_x, r_steer_y, steer_r)

        for i in xrange(self.const.WINDOW_SIZE):
            for j in xrange(self.const.WINDOW_SIZE):

                if active_grid[i,j] < self.const.C_MAX or (i == self.const.WINDOW_CENTER and j == self.const.WINDOW_CENTER):
                    # print "Skipped cel ({},{}) = {:.3f}".format(i,j, active_grid[i, j])
                    continue

                if self.active_window[i,j,BETA] == self.vcita:
                    print "Entering cell ({:d},{:d}) dead ahead".format(i,j)

                    # position of the cell
                    cij_x = (i+0.5)*self.const.RESOLUTION
                    cij_y = (j+0.5)*self.const.RESOLUTION

                    c_dist2 = (cij_x - l_steer_x)**2 + (cij_y - l_steer_y)**2
                    print "Left dist {:.3f}".format(c_dist2)
                    if c_dist2 < (self.const.R_RS + steer_l)**2:
                        phi_left = self.active_window[i,j,BETA] + 0.01
                        phi_left = phi_left - 2*math.pi if phi_left > math.pi else phi_left
                        print "Setting left limit angle to {:.1f} on cell ({:d},{:d}) = {:.1f}".format(phi_left*180.0/math.pi, i, j, self.active_window[i,j,MAG])

                    c_dist2 = (cij_x - r_steer_x)**2 + (cij_y - r_steer_y)**2
                    print "Right dist {:.3f}, limit {:.3f}".format(c_dist2, np.square(self.const.R_RS + steer_r))
                    if c_dist2 < (self.const.R_RS + steer_r)**2:
                        phi_right = self.active_window[i,j,BETA] - 0.01
                        phi_right = phi_right + 2*math.pi if phi_right < -math.pi else phi_right
                        print "Setting right limit angle to {:.1f} on cell ({:d},{:d})".format(phi_right*180.0/math.pi, i, j)


                elif self._isInRange(self.vcita, phi_left, self.active_window[i,j,BETA]):
                    #left
                    print "Entering cell ({:d},{:d}) in left range".format(i,j)

                    # position of the cell
                    cij_x = (i+0.5)*self.const.RESOLUTION
                    cij_y = (j+0.5)*self.const.RESOLUTION

                    # distance^2 from the cell to the steering center
                    c_dist2 = (cij_x - l_steer_x)**2 + (cij_y - l_steer_y)**2
                    print "Left dist {:.3f}".format(c_dist2)

                    if c_dist2 < (self.const.R_RS + steer_l)**2:
                        phi_left = self.active_window[i,j,BETA]
                        # phi_left = self.active_window[i,j,BETA] + 0.001
                        print "Setting left limit angle to {:.1f} on cell ({:d},{:d}) = {:.1f}".format(phi_left*180/math.pi, i, j, self.active_window[i,j,MAG])

                elif self._isInRange(phi_right, self.vcita, self.active_window[i,j,BETA]):
                    #right
                    print "Entering cell ({:d},{:d}) in right range".format(i,j)

                    # position of the cell
                    cij_x = (i+0.5)*self.const.RESOLUTION
                    cij_y = (j+0.5)*self.const.RESOLUTION

                    # distance^2 from the cell to the steering center
                    c_dist2 = (cij_x - r_steer_x)**2 + (cij_y - r_steer_y)**2
                    print "Right dist {:.3f}".format(c_dist2)

                    if c_dist2 < (self.const.R_RS + steer_r)**2:
                        phi_right = self.active_window[i,j,BETA]
                        # phi_right = phi_right + 2*math.pi if phi_right < -math.pi else phi_right
                        print "Setting right limit angle to {:.1f} on cell ({:d},{:d})".format(phi_right*180/math.pi, i, j)


        print "Limit angles:"
        print "Left:", phi_left*180.0/math.pi
        print "Right:", phi_right*180.0/math.pi
        for k in xrange(self.const.HIST_SIZE):
            if self.bin_polar_hist[k] == False:
                angle = k*self.const.ALPHA
                if angle >= math.pi:
                    angle -= 2*math.pi

                if self._isInRange(phi_right,phi_left, angle):
                    self.masked_polar_hist[k] = False
                else:
                    self.masked_polar_hist[k] = True
            else:
                self.masked_polar_hist[k] = True

    # This function determines if an angle in the range
    # [-pi, pi[ is inside the sector given, from start to
    # end (counter-clockwise), also angles in the range [-pi, pi[
    @staticmethod
    def _isInRange(start,end,angle):

        assert -math.pi <= start <= math.pi, 'start ({:.4f}) is not in range [-pi, pi]'.format(start)
        assert -math.pi <= end <= math.pi, 'end ({:.4f}) is not in range [-pi, pi]'.format(end)
        assert -math.pi <= angle <= math.pi, 'angle ({:.4f}) is not in range [-pi, pi]'.format(angle)

        if start < end:
            return (start <= angle and angle < end)
        else:
            return (start <= angle and angle <= math.pi) or (-math.pi <= angle and angle < end)


    def find_valleys(self):
        r"""Analiza el histograma polar mascarado y determina los
        valles candidatos.

        Los valles se obtienen a partir del
        :attr:`histograma polar mascarado<masked_polar_hist>`.

        Returns
        -------
        int
             * ``-1`` si no se encuentra ningún sector
               bloqueado en el hisotgrama polar mascarado.
             * De lo contrario, el número de valles encontrados.
        """

        start = None
        for x in xrange(self.const.HIST_SIZE):
            if self.masked_polar_hist[x]:
                start = x
                break

        # If no value was found over the threshold no action
        # needs to be taken since there are no nearby obstacles
        if start == None:
            return -1

        # Else, look for valleys after 'start'
        # True means blocked in the masked histogram
        # False means free
        self.valleys = []
        valley_found = False
        for i in xrange(self.const.HIST_SIZE+1):
            index = (start + i)%self.const.HIST_SIZE
            if not valley_found:
                if not self.masked_polar_hist[index]:
                    v_start = index
                    valley_found = True

            else:
                if self.masked_polar_hist[index]:
                    self.valleys.append(tuple([v_start, index-1]))
                    valley_found = False
        return len(self.valleys)

    def calculate_steering_dir(self, valley_count):
        r"""Calcula la dirección de movimiento para la evasión y
        la rapidez del robot.

        El cálculo de la dirección para el robot se hace a
        partir de los valles candidato, la orientación actual
        y el objetivo. Si no hay objetivo, se toma la dirección
        actual de movimiento como objetivo.
        Se usa una función de costo para seleccionar la dirección
        de entre todas las candidatas. La rapidez se calcula del
        costo de la dirección seleccionada, a mayor costo más
        lento se moverá el robot.

        Parameters
        ----------
        valley_count : int
            El resultado del llamado a :func:`find_valleys`.

        Returns
        -------
        new_dir  : float
            La dirección del movimiento, dada en radianes en el
            rango  :math:`[0°,\:360°[` respecto al marco de
            referencia global. 
        V : float
            La velocidad del robot.

        Raises
        ------
        Exception
            Si ``valley_count`` es cero (todas las direcciones
            están bloqueadas).

        Notes
        -----

        Los valles angostos definen una única dirección
        candidata, el medio del valle. Los anchos pueden definir
        hasta tres: los bordes del valle y la dirección del
        objetivo si esta se encuentra dentro del valle.

        """

        if valley_count == 0:
            raise Exception("No candidate valleys found to calculate a new direction")

        t_dir = None
        if self.target != None:
            # If there is a target for the robot we calculate the direction
            t_dir = math.atan2(self.target[1]-self.y_0, self.target[0]-self.x_0)
            # t_dir = t_dir if t_dir >= 0 else t_dir + 360
        else:
            # Else set the current direction as the target
            t_dir = self.vcita


        candidate_dirs = []

        print self.valleys
        if valley_count == -1:
            print "No obstacles nearby, setting route to target at {:.1f}".format(t_dir)
            candidate_dirs.append(t_dir)
        else:
            for v in self.valleys:
                s1, s2 = v
                v_size = (s2 - s1) if s2 >= s1 else self.const.HIST_SIZE - (s1-s2)

                if v_size < self.const.WIDE_V:
                    # Narrow valley
                    # The only target dir is the middle of
                    # the oppening
                    # print "narrow valley"
                    c_center = self.const.ALPHA*(s1 + 0.5 + v_size/2.0)
                    if c_center > math.pi:
                        c_center -= 2*math.pi
                    if c_center < -math.pi:
                        c_center += 2*math.pi

                    candidate_dirs.append(c_center)

                else:
                    # Wide valley
                    # Target dirs are the left and right
                    # borders,
                    # print "wide valley"
                    c_right = self.const.ALPHA*(s1 + 0.5 + self.const.WIDE_V/2.0)
                    if c_right > math.pi:
                        c_right -= 2*math.pi
                    if c_right < -math.pi:
                        c_right += 2*math.pi


                    c_left = self.const.ALPHA*(s2 + 0.5 - self.const.WIDE_V/2.0)
                    if c_left > math.pi:
                        c_left -= 2*math.pi
                    if c_left < -math.pi:
                        c_left += 2*math.pi

                    candidate_dirs.append(c_left)
                    candidate_dirs.append(c_right)

                    if c_right != c_left and self._isInRange(c_right,c_left,t_dir):
                        candidate_dirs.append(t_dir)

        # print candidate_dirs
        # Once all we know all possible candidate dirs
        # choose the one with the lowest cost
        new_dir = None
        best_cost = None
        for c in candidate_dirs:
            cost = self.const.MU1*self._abs_angle_diff(c, t_dir) + \
                    self.const.MU2*self._abs_angle_diff(c, self.vcita) + \
                    self.const.MU3*self._abs_angle_diff(c, self.prev_dir)
            # print "For candidate dir {:.1f}: {:.3f} cost".format(c,cost)

            if best_cost == None:
                new_dir = c
                best_cost = cost
            elif cost < best_cost:
                new_dir = c
                best_cost = cost

        self.prev_dir = new_dir
        self.prev_cost = best_cost

        V = self.const.V_MAX*(1 - best_cost/self.const.MAX_COST) + self.const.V_MIN

        # print "Setting dir to {:.1f}".format(new_dir)
        return new_dir, V

    @staticmethod
    def _abs_angle_diff(a1, a2):
        return min(2*math.pi - abs(a1 - a2), abs(a1 - a2))

    def calculate_speed(self):

        V = self.const.V_MAX*(1 - self.prev_cost/self.const.MAX_COST) + self.const.V_MIN
        return V

    @staticmethod
    def _dist(array,i,j):
        n_dist = abs(i-j)
        return min(n_dist, len(array) - n_dist)
