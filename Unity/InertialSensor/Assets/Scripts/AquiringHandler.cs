using System;

namespace AssemblyCSharp
{
	public class AquiringHandler
	{
		public AquiringHandler ()
		{
		}
		public void Beta()
		{
			while (true) {
				Console.WriteLine("Alpha.Beta is running in its own thread.");
			}
		}
	}
}

