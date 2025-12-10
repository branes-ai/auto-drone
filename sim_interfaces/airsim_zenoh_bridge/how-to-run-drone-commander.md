\# Session Summary: AirSim Zenoh Bridge



\## What We Built



Files created/modified in sim\_interfaces/airsim\_zenoh\_bridge/:



&nbsp; | File                   | Description                                            |

&nbsp; |------------------------|--------------------------------------------------------|

&nbsp; | airsim\_zenoh\_bridge.py | Main bridge - connects Project AirSim to Zenoh network |

&nbsp; | drone\_commander.py     | NEW - Command script to fly drone from Linux           |

&nbsp; | mock\_bridge.py         | NEW - Local testing without AirSim                     |

&nbsp; | data\_types.py          | Binary serialization (already existed)                 |



\## Capabilities Verified



&nbsp; | Feature                                  | Status             |

&nbsp; |------------------------------------------|--------------------|

&nbsp; | Velocity commands (Linux→Windows→AirSim) | ✅                  |

&nbsp; | Odometry feedback (AirSim→Windows→Linux) | ✅                  |

&nbsp; | RGB camera streaming                     | ✅ 400x225 @ ~21fps |

&nbsp; | Depth camera streaming                   | ✅ 400x225 @ ~21fps |

&nbsp; | Square mission                           | ✅                  |

&nbsp; | Circle mission                           | ✅                  |

&nbsp; | Up/down mission                          | ✅                  |



\## Usage



&nbsp; Windows (run the bridge):

```bash

&nbsp; cd sim\_interfaces\\airsim\_zenoh\_bridge

&nbsp; python airsim\_zenoh\_bridge.py --zenoh-listen tcp/0.0.0.0:7447 --auto-takeoff --scene scene\_basic\_drone.jsonc

```



&nbsp; Linux (send commands):

```bash

&nbsp; # Missions

&nbsp; python3 drone\_commander.py --connect tcp/192.168.1.10:7447 --mission square

&nbsp; python3 drone\_commander.py --connect tcp/192.168.1.10:7447 --mission circle

&nbsp; python3 drone\_commander.py --connect tcp/192.168.1.10:7447 --mission updown

```



&nbsp; # Interactive

```bash

&nbsp; python3 drone\_commander.py --connect tcp/192.168.1.10:7447

```



---



\## Next Steps



Ready to integrate with the existing demos:

&nbsp; 1. Waypoint following demo (demos/02\_waypoint\_following/) - use real AirSim odometry

&nbsp; 2. Object tracking demo (demos/03\_object\_tracking/) - use real AirSim camera images



