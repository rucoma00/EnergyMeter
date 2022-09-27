extern char tx_bufi[];

void hexdec_int(unsigned int counti);

    char onesi;
	char tensi;
	char hundredsi;
	char thousandsi;
	char thousand10si;
	
 void hexdec_int( unsigned int counti )
{	
	thousand10si=0;
    thousandsi=0;
	hundredsi=0;
	tensi  = 0;						
	onesi = 0;

			while ( counti >= 10000 )
			{
				counti -= 10000;			// subtract 1000
		
				thousand10si++;					// increment hundreds
			}

			while ( counti >= 1000 )
			{
				counti -= 1000;			// subtract 1000
		
				thousandsi++;					// increment hundreds
			}

			while ( counti >= 100 )
			{
				counti -= 100;			// subtract 100
		
				hundredsi++;					// increment hundreds
			}


				while ( counti >= 10 )
					{
						counti -= 10;			// subtract 10
		
						tensi++;					// increment tens
					}

						onesi = counti;					// remaining count equals ones
						
					tx_bufi[4]=thousand10si+0x30;
                	tx_bufi[3]=thousandsi+0x30;
					tx_bufi[2]=hundredsi+0x30;
					tx_bufi[1]=tensi+0x30;
					tx_bufi[0]=onesi+0x30;	
}
 
