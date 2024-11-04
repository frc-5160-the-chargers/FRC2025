package frc.chargers.utils.units

import com.batterystaple.kmeasure.dimensions.*
import com.batterystaple.kmeasure.quantities.Quantity

typealias VoltageRate = Quantity<VoltageRateDimension>
typealias VoltageRateDimension = Dimension<Mass1, Length2, TimeN4, CurrentN1>