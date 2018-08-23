///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains a simple container class.
 *	\file		IceContainer.h
 *	\author		Pierre Terdiman
 *	\date		February, 5, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __ICECONTAINER_H__
#define __ICECONTAINER_H__

	#define CONTAINER_STATS

	enum FindMode
	{
		FIND_CLAMP,
		FIND_WRAP,

		FIND_FORCE_DWORD = 0x7fffffff
	};

	struct ICECORE_API contactStruct {
		int primitiveType;
		// for edge-edge
		std::array<int, 2> edgePair;			// edge pair
		//std::array<float, 3>  normals;			// only for edge-edge
		//// for node-triangle, triangle-node
		//std::array<float, 3> penetrations;		// 3 penetration of a node-triangle pair
		
	};
	
	class ICECORE_API Container
	{
		public:
		// Constructor / Destructor
								Container();
								Container(const Container& object);
								Container(udword size, double growth_factor);
								~Container();
		// Management
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	A O(1) method to add a value in the container. The container is automatically resized if needed.
		 *	The method is inline, not the resize. The call overhead happens on resizes only, which is not a problem since the resizing operation
		 *	costs a lot more than the call overhead...
		 *
		 *	\param		entry		[in] a udword to store in the container
		 *	\see		Add(float entry)
		 *	\see		Empty()
		 *	\see		Contains(udword entry)
		 *	\return		Self-Reference
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_	Container&		Add(udword entry)
								{
									// Resize if needed
									if(mCurNbEntries==mMaxNbEntries)	Resize();

									// Add new entry
									mEntries[mCurNbEntries++]	= entry;
									return *this;
								}

		inline_	Container&		Add(const uword* entries, udword nb)
								{
									// Resize if needed
									if(mCurNbEntries+nb>mMaxNbEntries)	Resize(nb);

									// Add new entry
									CopyMemory(&mEntries[mCurNbEntries], entries, nb*sizeof(uword));
									mCurNbEntries+=nb;
									return *this;
								}

		inline_	Container&		Add(const udword* entries, udword nb)
								{
									// Resize if needed
									if(mCurNbEntries+nb>mMaxNbEntries)	Resize(nb);

									// Add new entry
									CopyMemory(&mEntries[mCurNbEntries], entries, nb*sizeof(udword));
									mCurNbEntries+=nb;
									return *this;
								}

		inline_ void			ContactArgAdd(IceCore::contactStruct &entry)
		{
			contactArgs.push_back(entry);
		}

		inline_ const std::vector<IceCore::contactStruct> &GetContactArgs() const { return contactArgs; }
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	A O(1) method to add a value in the container. The container is automatically resized if needed.
		 *	The method is inline, not the resize. The call overhead happens on resizes only, which is not a problem since the resizing operation
		 *	costs a lot more than the call overhead...
		 *
		 *	\param		entry		[in] a float to store in the container
		 *	\see		Add(udword entry)
		 *	\see		Empty()
		 *	\see		Contains(udword entry)
		 *	\return		Self-Reference
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//inline_	Container&		Add(float entry)
		//						{
		//							// Resize if needed
		//							if(mCurNbEntries==mMaxNbEntries)	Resize();

		//							// Add new entry
		//							mEntries[mCurNbEntries++]	= IR(entry);
		//							return *this;
		//						}

		//inline_	Container&		Add(const float* entries, udword nb)
		//						{
		//							// Resize if needed
		//							if(mCurNbEntries+nb>mMaxNbEntries)	Resize(nb);

		//							// Add new entry
		//							CopyMemory(&mEntries[mCurNbEntries], entries, nb*sizeof(float));
		//							mCurNbEntries+=nb;
		//							return *this;
		//						}

		//! Add unique [slow]
		inline_	Container&		AddUnique(udword entry)
								{
									if(!Contains(entry))	Add(entry);
									return *this;
								}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Clears the container. All stored values are deleted, and it frees used ram.
		 *	\see		Reset()
		 *	\return		Self-Reference
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				Container&		Empty();

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Resets the container. Stored values are discarded but the buffer is kept so that further calls don't need resizing again.
		 *	That's a kind of temporal coherence.
		 *	\see		Empty()
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		inline_	void			Reset()
								{
									// Avoid the write if possible
									// ### CMOV
									if(mCurNbEntries)	mCurNbEntries = 0;
									contactArgs.clear();
								}

		// HANDLE WITH CARE
		inline_	void			ForceSize(udword size)
								{
									mCurNbEntries = size;
								}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Sets the initial size of the container. If it already contains something, it's discarded.
		 *	\param		nb		[in] Number of entries
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				bool			SetSize(udword nb);

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Refits the container and get rid of unused bytes.
		 *	\return		true if success
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				bool			Refit();

		// Checks whether the container already contains a given value.
				bool			Contains(udword entry, udword* location=null) const;
		// Deletes an entry - doesn't preserve insertion order.
				bool			Delete(udword entry);
		// Deletes an entry - does preserve insertion order.
				bool			DeleteKeepingOrder(udword entry);
		//! Deletes the very last entry.
		inline_	void			DeleteLastEntry()						{ if(mCurNbEntries)	mCurNbEntries--;			}
		//! Deletes the entry whose index is given
		inline_	void			DeleteIndex(udword index)				{ mEntries[index] = mEntries[--mCurNbEntries];	}

		// Helpers
				Container&		FindNext(udword& entry, IceCore::FindMode find_mode= IceCore::FindMode::FIND_CLAMP);
				Container&		FindPrev(udword& entry, IceCore::FindMode find_mode= IceCore::FindMode::FIND_CLAMP);
		// Data access.
		inline_	udword			GetNbEntries()					const	{ return mCurNbEntries;					}	//!< Returns the current number of entries.
		inline_	udword			GetEntry(udword i)				const	{ return mEntries[i];					}	//!< Returns ith entry
		inline_	udword*			GetEntries()					const	{ return mEntries;						}	//!< Returns the list of entries.

		inline_	udword			GetFirst()						const	{ return mEntries[0];					}
		inline_	udword			GetLast()						const	{ return mEntries[mCurNbEntries-1];		}

		// Growth control
		inline_	double			GetGrowthFactor()				const	{ return mGrowthFactor;					}	//!< Returns the growth factor
		inline_	void			SetGrowthFactor(double growth)			{ mGrowthFactor = growth;				}	//!< Sets the growth factor
		inline_	bool			IsFull()						const	{ return mCurNbEntries==mMaxNbEntries;	}	//!< Checks the container is full
		inline_	BOOL			IsNotEmpty()					const	{ return mCurNbEntries;					}	//!< Checks the container is empty

		//! Read-access as an array
		inline_	udword			operator[](udword i)			const	{ ASSERT(i>=0 && i<mCurNbEntries); return mEntries[i];	}
		//! Write-access as an array
		inline_	udword&			operator[](udword i)					{ ASSERT(i>=0 && i<mCurNbEntries); return mEntries[i];	}

		// Stats
				udword			GetUsedRam()					const;

		//! Operator for "Container A = Container B"
				//void			operator = (const Container& object);

#ifdef CONTAINER_STATS
		inline_	udword			GetNbContainers()				const	{ return mNbContainers;		}
		inline_	udword			GetTotalBytes()					const	{ return mUsedRam;			}
		private:

		static	udword			mNbContainers;		//!< Number of containers around
		static	udword			mUsedRam;			//!< Amount of bytes used by containers in the system
#endif

		protected:
		// Resizing
				bool			Resize(udword needed=1);
		// Data
				udword			mMaxNbEntries;		//!< Maximum possible number of entries
				udword			mCurNbEntries;		//!< Current number of entries
				udword*			mEntries;			//!< List of entries
				double			mGrowthFactor;		//!< Resize: new number of entries = old number * mGrowthFactor
	
		// my extended data and function
				std::vector<IceCore::contactStruct> contactArgs;
	};


	//class ICECORE_API qContainer : public Container
	//{
	//protected:
	//	uqword*			mEntries;

	//public:

	//	//////////////////////////////////////////////////////////
	//	// overload some functions that associate uqword
	//	inline_	qContainer&		Add(double entry)
	//	{
	//		// Resize if needed
	//		if (mCurNbEntries == mMaxNbEntries)	Resize();

	//		// Add new entry
	//		mEntries[mCurNbEntries++] = IR(entry);
	//		return *this;
	//	}

	//	qContainer& Empty();

	//	bool SetSize(udword nb);

	//	bool Refit();

	//	bool Contains(uqword entry, udword* location = null) const;

	//	bool			Delete(uqword entry);

	//	bool			DeleteKeepingOrder(uqword entry);

	//	qContainer&		FindNext(uqword& entry, IceCore::FindMode find_mode = IceCore::FindMode::FIND_CLAMP);
	//	qContainer&		FindPrev(uqword& entry, IceCore::FindMode find_mode = IceCore::FindMode::FIND_CLAMP);

	//	inline_	uqword			GetEntry(udword i)				const { return mEntries[i]; }	//!< Returns ith entry
	//	inline_	uqword*			GetEntries()					const { return mEntries; }	//!< Returns the list of entries.
	//	inline_	uqword			GetFirst()						const { return mEntries[0]; }
	//	inline_	uqword			GetLast()						const { return mEntries[mCurNbEntries - 1]; }
	//	inline_	uqword			operator[](udword i)			const { ASSERT(i >= 0 && i<mCurNbEntries); return mEntries[i]; }
	//	inline_	uqword&			operator[](udword i)				  { ASSERT(i >= 0 && i<mCurNbEntries); return mEntries[i]; }

	//};


#endif // __ICECONTAINER_H__
